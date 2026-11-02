#!/usr/bin/env python3
"""Drive four sides, rotating left after each side."""

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self._odom_received = False
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def on_odom(self, _msg):
        self._odom_received = True

    def _wait_for_servers(self):
        for label, client in (
            ("drive_distance", self.drive_client),
            ("rotate", self.rotate_client),
        ):
            while rclpy.ok() and not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for /{label} action server")
            if not rclpy.ok():
                return False
        return True

    def _send_goal(self, client, goal):
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def _wait_for_odometry(self):
        if not self._odom_received:
            self.get_logger().info("Waiting for /odom")
        while rclpy.ok() and not self._odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        return rclpy.ok()

    def _stop_and_report_failure(self, side, stage):
        self.cmd_vel.publish(Twist())
        self.get_logger().error(f"Square failed on side {side}: {stage} goal did not succeed")

    def drive_square(self):
        side_length = float(self.get_parameter("side_length").value)
        if side_length <= 0.0:
            self.get_logger().error("side_length must be greater than zero")
            return
        if not self._wait_for_servers() or not self._wait_for_odometry():
            return

        for side in range(1, 5):
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length
            drive_result = self._send_goal(self.drive_client, drive_goal)
            if (
                drive_result is None
                or drive_result.status != GoalStatus.STATUS_SUCCEEDED
                or not drive_result.result.reached
            ):
                self._stop_and_report_failure(side, "drive_distance")
                return

            rotate_goal = Rotate.Goal()
            rotate_goal.angle = 1.57079632679
            rotate_result = self._send_goal(self.rotate_client, rotate_goal)
            if (
                rotate_result is None
                or rotate_result.status != GoalStatus.STATUS_SUCCEEDED
            ):
                self._stop_and_report_failure(side, "rotate")
                return

        self.get_logger().info(f"Completed a square with {side_length:.2f} m sides")


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.drive_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
