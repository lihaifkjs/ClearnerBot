"""ROS2 桥接:订阅 /cmd_vel 翻译成现有运动控制 API;发布 /odom + TF。

契约(PRD §10.3):
- M4:消费 /cmd_vel(geometry_msgs/Twist,linear.x m/s、angular.z rad/s),
  除以标称最大速度换算为归一化 (vx, wz) 交给上层(handle_drive),
  越界由 core/motion.mix 的 clamp/峰值等比缩小兜底。
- M7:发布 /odom(nav_msgs/Odometry,frame odom→base_link)与同名 TF,
  数据源为注入的 get_odom 回调(odometry.as_dict),由控制循环节拍驱动。

嵌入形态(M4 方案 §2):rclpy 与车端服务同进程,后台线程 spin;
GPIO 保持单实例拥有。rclpy 延迟导入 —— 无 ROS 的环境(Mac 开发机、
离线测试)start() 返回 False,服务其余功能不受影响。
"""
import json
import logging
import math
import threading
from dataclasses import dataclass, fields
from pathlib import Path

log = logging.getLogger(__name__)

ROBOT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "robot.json"


@dataclass
class RobotParams:
    """桥接换算用的标称最大速度(方案参数,入库;与机器标定 calibration.json 区分)。"""

    max_linear_speed_mps: float = 0.5  # 占位初值,M5 mock 标称轮速,真机实测后修正
    max_angular_speed_radps: float = 7.0  # 占位初值,按轮距 141.7mm 推算,真机实测后修正

    @classmethod
    def load(cls, path=ROBOT_CONFIG_PATH):
        """读取配置;文件缺失/损坏 → 默认初值,多余字段忽略。"""
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return cls()
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})


def twist_to_normalized(linear_x, angular_z, params):
    """Twist 物理单位 → 归一化 (vx, wz)。纯函数,不依赖 rclpy。"""
    return (
        linear_x / params.max_linear_speed_mps,
        angular_z / params.max_angular_speed_radps,
    )


def yaw_to_quaternion(yaw):
    """偏航角(rad)→ 四元数 (x, y, z, w)。纯函数;平面底盘只有 z 轴旋转。"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _import_rclpy():
    """延迟导入 rclpy 与 Twist;无 ROS 环境返回 None。独立成函数便于测试替换。"""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ImportError:
        return None
    return rclpy, Twist


def _import_odom_support():
    """延迟导入 /odom 发布依赖;缺包返回 None(此时 /cmd_vel 桥接仍可用)。"""
    try:
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from tf2_ros import TransformBroadcaster
    except ImportError:
        return None
    return Odometry, TransformStamped, TransformBroadcaster


class CmdVelBridge:
    """/cmd_vel 订阅节点。回调在 ROS spin 线程执行,经 on_drive 调用上层;

    上层(handle_drive → drivetrain.set_speeds)被控制循环与本线程并发调用,
    gpiozero/lgpio 的引脚写是原子操作,且 set_speeds 无语义状态,可安全并发。
    """

    def __init__(self, on_drive, params, topic="/cmd_vel", get_odom=None):
        self._on_drive = on_drive
        self._params = params
        self._topic = topic
        self._get_odom = get_odom
        self._rclpy = None
        self._node = None
        self._thread = None
        self._odom_pub = None
        self._tf_broadcaster = None
        self._odom_types = None  # (Odometry, TransformStamped),start 时填充

    def start(self):
        """启动节点;无 ROS 环境或已启动时返回 False。"""
        if self._node is not None:
            return False
        modules = _import_rclpy()
        if modules is None:
            log.warning("rclpy 不可用(非 ROS 环境),/cmd_vel 桥接未启动")
            return False
        self._rclpy, Twist = modules
        self._rclpy.init()
        self._node = self._rclpy.create_node("clearnerbot_bridge")
        self._node.create_subscription(Twist, self._topic, self._on_twist, 10)
        if self._get_odom is not None:
            self._setup_odom_pub()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        log.info("ROS2 桥接已启动,订阅 %s", self._topic)
        return True

    def _setup_odom_pub(self):
        """建 /odom publisher 与 TF broadcaster;缺 nav_msgs/tf2_ros 时跳过。"""
        modules = _import_odom_support()
        if modules is None:
            log.warning("nav_msgs/tf2_ros 不可用,/odom 发布未启动")
            return
        Odometry, TransformStamped, broadcaster_cls = modules
        self._odom_types = (Odometry, TransformStamped)
        self._odom_pub = self._node.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = broadcaster_cls(self._node)
        log.info("/odom 发布已启动(odom → base_link)")

    def publish_odom(self):
        """发布当前里程计为 /odom + odom→base_link TF;未启动时静默跳过。

        由控制循环 50ms 节拍调用(与里程计积分同节拍,约 20Hz)。
        """
        if self._odom_pub is None:
            return
        Odometry, TransformStamped = self._odom_types
        odom = self._get_odom()
        stamp = self._node.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(odom["theta"])

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = odom["x"]
        msg.pose.pose.position.y = odom["y"]
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 协方差为占位值(M7 方案 §4.1):slam_toolbox online_async 不依赖它
        msg.pose.covariance[0] = 0.01  # x
        msg.pose.covariance[7] = 0.01  # y
        msg.pose.covariance[35] = 0.01  # yaw
        msg.twist.twist.linear.x = odom["vx"]
        msg.twist.twist.angular.z = odom["wz"]
        msg.twist.covariance[0] = 0.01  # vx
        msg.twist.covariance[35] = 0.01  # wz
        self._odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = odom["x"]
        tf.transform.translation.y = odom["y"]
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def _spin(self):
        try:
            self._rclpy.spin(self._node)
        except Exception:  # stop() 触发 shutdown 唤醒 spin 抛异常,属正常退出路径
            pass

    def _on_twist(self, msg):
        vx, wz = twist_to_normalized(msg.linear.x, msg.angular.z, self._params)
        self._on_drive(vx, wz)

    def stop(self):
        """停节点并回收线程;幂等,允许重复调用。"""
        if self._node is None:
            return
        self._node.destroy_node()
        self._rclpy.shutdown()
        self._thread.join(timeout=2)
        self._node = None
        self._rclpy = None
        self._thread = None
        self._odom_pub = None
        self._tf_broadcaster = None
        self._odom_types = None
