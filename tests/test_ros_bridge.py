"""comm/ros_bridge 测试:换算纯函数、参数加载、节点收发与生命周期(假 rclpy)。"""
import threading
from types import SimpleNamespace

import pytest

from src.comm import ros_bridge
from src.comm.ros_bridge import CmdVelBridge, RobotParams, twist_to_normalized


@pytest.mark.parametrize(
    "linear_x, angular_z, expected",
    [
        (0.25, 3.5, (0.5, 0.5)),  # 半速
        (0.0, 0.0, (0.0, 0.0)),
        (-0.5, -7.0, (-1.0, -1.0)),  # 满速后退 + 满速右转
        (1.0, 14.0, (2.0, 2.0)),  # 越界不在此截断,交给 mix 峰值等比缩小
    ],
)
def test_twist_to_normalized(linear_x, angular_z, expected):
    vx, wz = twist_to_normalized(linear_x, angular_z, RobotParams())
    assert (vx, wz) == pytest.approx(expected)


def test_params_default_when_file_missing(tmp_path):
    assert RobotParams.load(tmp_path / "nope.json") == RobotParams()


def test_params_load_and_ignore_extra_fields(tmp_path):
    path = tmp_path / "robot.json"
    path.write_text('{"max_linear_speed_mps": 0.3, "extra": 1}', encoding="utf-8")
    params = RobotParams.load(path)
    assert params.max_linear_speed_mps == 0.3
    assert params.max_angular_speed_radps == RobotParams().max_angular_speed_radps


class _FakeTwist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class _FakeNode:
    def __init__(self):
        self.subscription = None
        self.destroyed = False

    def create_subscription(self, msg_type, topic, callback, depth):
        self.subscription = (topic, callback)

    def destroy_node(self):
        self.destroyed = True


class _FakeRclpy:
    """最小假 rclpy:spin 阻塞直到 shutdown,模拟真实线程模型。"""

    def __init__(self):
        self.node = _FakeNode()
        self._woken = threading.Event()

    def init(self):
        pass

    def create_node(self, name):
        return self.node

    def spin(self, node):
        self._woken.wait(timeout=2)

    def shutdown(self):
        self._woken.set()


def _patch_rclpy(monkeypatch):
    fake = _FakeRclpy()
    monkeypatch.setattr(ros_bridge, "_import_rclpy", lambda: (fake, _FakeTwist))
    return fake


def test_bridge_delivers_twist_as_normalized_drive(monkeypatch):
    fake = _patch_rclpy(monkeypatch)
    received = []
    bridge = CmdVelBridge(lambda vx, wz: received.append((vx, wz)), RobotParams())

    assert bridge.start() is True
    topic, callback = fake.node.subscription
    assert topic == "/cmd_vel"

    msg = _FakeTwist()
    msg.linear.x = 0.25
    msg.angular.z = -3.5
    callback(msg)
    assert received == [(0.5, -0.5)]

    bridge.stop()
    assert fake.node.destroyed
    bridge.stop()  # 幂等


def test_start_twice_returns_false(monkeypatch):
    _patch_rclpy(monkeypatch)
    bridge = CmdVelBridge(lambda vx, wz: None, RobotParams())
    assert bridge.start() is True
    assert bridge.start() is False
    bridge.stop()


def test_start_without_rclpy_returns_false(monkeypatch):
    monkeypatch.setattr(ros_bridge, "_import_rclpy", lambda: None)
    bridge = CmdVelBridge(lambda vx, wz: None, RobotParams())
    assert bridge.start() is False
    bridge.stop()  # 未启动时 stop 安全
