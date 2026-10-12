"""comm/ros_bridge 测试:换算纯函数、参数加载、节点收发与生命周期(假 rclpy)。"""
import math
import threading
from types import SimpleNamespace

import pytest

from src.comm import ros_bridge
from src.comm.ros_bridge import (
    CmdVelBridge,
    RobotParams,
    twist_to_normalized,
    yaw_to_quaternion,
)


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


@pytest.mark.parametrize(
    "yaw, expected",
    [
        (0.0, (0.0, 0.0, 0.0, 1.0)),
        (math.pi / 2, (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))),
        (math.pi, (0.0, 0.0, 1.0, 0.0)),
        (-math.pi / 2, (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))),
    ],
)
def test_yaw_to_quaternion(yaw, expected):
    assert yaw_to_quaternion(yaw) == pytest.approx(expected)


class _FakeTwist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class _FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeNode:
    def __init__(self):
        self.subscription = None
        self.publishers = []
        self.destroyed = False

    def create_subscription(self, msg_type, topic, callback, depth):
        self.subscription = (topic, callback)

    def create_publisher(self, msg_type, topic, depth):
        pub = _FakePublisher(topic)
        self.publishers.append(pub)
        return pub

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "STAMP"))

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


class _FakeOdometry:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        )
        self.twist = SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0),
                angular=SimpleNamespace(z=0.0),
            ),
            covariance=[0.0] * 36,
        )


class _FakeTransformStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.transform = SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class _FakeBroadcaster:
    def __init__(self, _node):
        self.sent = []

    def sendTransform(self, tf):
        self.sent.append(tf)


def _patch_odom_support(monkeypatch):
    monkeypatch.setattr(
        ros_bridge,
        "_import_odom_support",
        lambda: (_FakeOdometry, _FakeTransformStamped, _FakeBroadcaster),
    )


_ODOM_SAMPLE = {"x": 1.0, "y": 0.5, "theta": math.pi / 2, "vx": 0.2, "wz": 0.1}


def test_publish_odom_sends_odometry_and_tf(monkeypatch):
    fake = _patch_rclpy(monkeypatch)
    _patch_odom_support(monkeypatch)
    bridge = CmdVelBridge(lambda vx, wz: None, RobotParams(),
                          get_odom=lambda: dict(_ODOM_SAMPLE))

    assert bridge.start() is True
    bridge.publish_odom()

    pub = fake.node.publishers[0]
    assert pub.topic == "/odom"
    msg = pub.published[0]
    assert msg.header.stamp == "STAMP"
    assert (msg.header.frame_id, msg.child_frame_id) == ("odom", "base_link")
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, 0.5)
    assert msg.pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert msg.pose.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))
    assert (msg.twist.twist.linear.x, msg.twist.twist.angular.z) == (0.2, 0.1)

    tf = bridge._tf_broadcaster.sent[0]
    assert (tf.header.frame_id, tf.child_frame_id) == ("odom", "base_link")
    assert (tf.transform.translation.x, tf.transform.translation.y) == (1.0, 0.5)
    assert tf.transform.rotation.w == pytest.approx(math.cos(math.pi / 4))
    bridge.stop()


def test_publish_odom_noop_when_unavailable(monkeypatch):
    # 未启动:静默跳过
    bridge = CmdVelBridge(lambda vx, wz: None, RobotParams(), get_odom=lambda: {})
    bridge.publish_odom()

    # 缺 nav_msgs/tf2_ros:仅 /cmd_vel 可用,publish_odom 静默跳过
    _patch_rclpy(monkeypatch)
    monkeypatch.setattr(ros_bridge, "_import_odom_support", lambda: None)
    assert bridge.start() is True
    bridge.publish_odom()
    bridge.stop()
