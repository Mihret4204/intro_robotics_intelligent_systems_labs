#!/usr/bin/env python3
"""Rotate the robot by a requested signed angle, measured from odometry."""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self._odom_lock = threading.Lock()
        self._yaw = None
        self._callback_group = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            10,
            callback_group=self._callback_group,
        )
        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self._callback_group,
        )

    def on_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._odom_lock:
            self._yaw = yaw

    def on_goal(self, goal):
        if not math.isfinite(goal.angle) or goal.angle == 0.0:
            return GoalResponse.REJECT
        with self._odom_lock:
            if self._yaw is None:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        goal = float(goal_handle.request.angle)
        with self._odom_lock:
            previous_yaw = self._yaw
        if previous_yaw is None:
            goal_handle.abort()
            return Rotate.Result()

        result = Rotate.Result()
        feedback = Rotate.Feedback()
        turned = 0.0
        direction = 1.0 if goal > 0.0 else -1.0
        turn_speed = abs(float(self.get_parameter("turn_speed").value))
        twist = Twist()
        twist.angular.z = direction * turn_speed

        self.get_logger().info(f"Rotating {goal:.3f} rad")
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.cmd_vel.publish(Twist())
                    result.turned = turned
                    goal_handle.canceled()
                    self.get_logger().info(f"Rotation canceled after {turned:.3f} rad")
                    return result

                with self._odom_lock:
                    current_yaw = self._yaw
                if current_yaw is not None:
                    turned += angle_difference(current_yaw, previous_yaw)
                    previous_yaw = current_yaw

                feedback.turned = turned
                goal_handle.publish_feedback(feedback)
                if goal * turned >= goal * goal:
                    break

                self.cmd_vel.publish(twist)
                time.sleep(0.05)

            result.turned = turned
            if rclpy.ok():
                goal_handle.succeed()
                self.get_logger().info(f"Rotated {turned:.3f} rad")
            else:
                goal_handle.abort()
            return result
        finally:
            self.cmd_vel.publish(Twist())


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
