#!/usr/bin/env python3
"""Turn away from nearby obstacles when enabled through a service."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        self.enabled = False
        self.last_obstacle_distance = float("inf")

        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_status = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

    def on_obstacle(self, msg):
        self.last_obstacle_distance = msg.distance
        status = RobotStatus()
        status.obstacle_distance = msg.distance

        if not self.enabled:
            status.state = "stopped"
        elif msg.too_close:
            status.state = "turning"
            twist = Twist()
            turn_speed = float(self.get_parameter("turn_speed").value)
            twist.angular.z = -turn_speed if msg.angle > 0.0 else turn_speed
            self.cmd_vel.publish(twist)
        else:
            status.state = "forward"
            twist = Twist()
            twist.linear.x = float(self.get_parameter("forward_speed").value)
            self.cmd_vel.publish(twist)

        self.robot_status.publish(status)

    def on_enable(self, request, response):
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "enabled"
        else:
            if self.enabled:
                self.cmd_vel.publish(Twist())
            self.enabled = False
            response.success = True
            response.message = "stopped"
            status = RobotStatus()
            status.state = "stopped"
            status.obstacle_distance = self.last_obstacle_distance
            self.robot_status.publish(status)

        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = float(self.get_parameter("max_turn_speed").value)
        if not 0.0 < request.turn_speed <= max_turn_speed:
            response.success = False
            response.message = f"turn_speed must be above 0 and at most {max_turn_speed} rad/s"
        else:
            self.set_parameters([
                Parameter("turn_speed", value=float(request.turn_speed))
            ])
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed:.2f} rad/s"

        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
