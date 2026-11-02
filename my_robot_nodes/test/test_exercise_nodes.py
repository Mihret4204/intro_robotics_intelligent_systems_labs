import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from my_robot_interfaces.srv import SetTurnSpeed
from my_robot_nodes.rotate_server import angle_difference
from my_robot_nodes.turn_away import TurnAway


class RecordingPublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def turn_away_node():
    node = TurnAway()
    node.cmd_vel = RecordingPublisher()
    node.robot_status = RecordingPublisher()
    yield node
    node.destroy_node()


def obstacle(*, distance=1.0, angle=0.0, too_close=False):
    message = ObstacleInfo()
    message.distance = distance
    message.angle = angle
    message.too_close = too_close
    return message


def test_starts_disabled_and_reports_stopped(turn_away_node):
    turn_away_node.on_obstacle(obstacle(distance=2.0))

    assert turn_away_node.cmd_vel.messages == []
    status = turn_away_node.robot_status.messages[-1]
    assert status.state == "stopped"
    assert status.obstacle_distance == pytest.approx(2.0)


def test_forward_turn_direction_and_live_speed_parameters(turn_away_node):
    enable = SetBool.Request()
    enable.data = True
    turn_away_node.on_enable(enable, SetBool.Response())

    turn_away_node.on_obstacle(obstacle())
    assert turn_away_node.cmd_vel.messages[-1].linear.x == pytest.approx(0.2)
    assert turn_away_node.robot_status.messages[-1].state == "forward"

    turn_away_node.set_parameters([
        Parameter("forward_speed", value=0.4),
        Parameter("turn_speed", value=0.7),
    ])
    turn_away_node.on_obstacle(obstacle(too_close=True, angle=0.3))
    left_obstacle_twist = turn_away_node.cmd_vel.messages[-1]
    assert left_obstacle_twist.linear.x == 0.0
    assert left_obstacle_twist.angular.z == pytest.approx(-0.7)
    assert turn_away_node.robot_status.messages[-1].state == "turning"

    turn_away_node.on_obstacle(obstacle(too_close=True, angle=-0.3))
    assert turn_away_node.cmd_vel.messages[-1].angular.z == pytest.approx(0.7)

    turn_away_node.on_obstacle(obstacle())
    assert turn_away_node.cmd_vel.messages[-1].linear.x == pytest.approx(0.4)


def test_turn_speed_service_validates_and_disable_stops_once(turn_away_node):
    request = SetTurnSpeed.Request()
    request.turn_speed = 1.0
    response = turn_away_node.on_set_turn_speed(request, SetTurnSpeed.Response())
    assert response.success
    assert turn_away_node.get_parameter("turn_speed").value == pytest.approx(1.0)

    request.turn_speed = 3.0
    response = turn_away_node.on_set_turn_speed(request, SetTurnSpeed.Response())
    assert not response.success
    assert turn_away_node.get_parameter("turn_speed").value == pytest.approx(1.0)

    enable = SetBool.Request()
    enable.data = True
    turn_away_node.on_enable(enable, SetBool.Response())
    disable = SetBool.Request()
    disable.data = False
    turn_away_node.on_enable(disable, SetBool.Response())
    assert isinstance(turn_away_node.cmd_vel.messages[-1], Twist)
    assert turn_away_node.cmd_vel.messages[-1] == Twist()
    stop_count = len(turn_away_node.cmd_vel.messages)

    turn_away_node.on_enable(disable, SetBool.Response())
    turn_away_node.on_obstacle(obstacle())
    assert len(turn_away_node.cmd_vel.messages) == stop_count
    assert turn_away_node.robot_status.messages[-1].state == "stopped"


def test_angle_difference_wraps_at_pi():
    delta = angle_difference(-math.pi + 0.01, math.pi - 0.01)
    assert delta == pytest.approx(0.02)
