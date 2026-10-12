"""LD06 -> /scan 独立发布节点。

用法(树莓派,RoboStack 环境):
    ~/miniforge3/envs/ros/bin/python -m src.lidar.ld06_node

rclpy/pyserial 只在 main() 内延迟导入,离线环境 import 本模块安全;
ScanAssembler 等组装逻辑为纯 Python,可离线单测。
"""

from __future__ import annotations

import math

from .ld06 import RANGE_MAX_M, RANGE_MIN_M, Ld06Packet, Ld06Parser

BINS = 360  # 固定 1° 分辨率,slam_toolbox 友好

DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 230400
DEFAULT_FRAME_ID = "laser_link"


def ld_to_ros_bin(angle_deg: float) -> int:
    """LD06 角度(正前方 0°,顺时针) -> LaserScan bin(前方 0,逆时针)。

    ROS 角度正方向为逆时针,与雷达相反,故取 (360 - angle);两个坐标系
    的 0° 都是车头正前方。
    """
    return int(round((360.0 - angle_deg) % 360.0)) % BINS


class ScanAssembler:
    """把连续的 LD06 包按"起始角回落"切成整圈,组装定长 ranges/intensities。

    无效点(距离为 0 或超出量程)对应 bin 保持 inf(LaserScan 惯例);
    多点落入同一 bin 时取最近距离(抗重扫抖动)。
    """

    def __init__(self) -> None:
        self._ranges = [math.inf] * BINS
        self._intensities = [0.0] * BINS
        self._last_start: float | None = None
        self._speed_dps = 0.0

    def add_packet(self, pkt: Ld06Packet) -> tuple[list[float], list[float], float] | None:
        """喂入一包;若凑满一圈,返回 (ranges, intensities, speed_dps),否则 None。"""
        scan = None
        start = pkt.points[0][0]
        if self._last_start is not None and start < self._last_start:
            scan = (self._ranges, self._intensities, self._speed_dps)
            self._ranges = [math.inf] * BINS
            self._intensities = [0.0] * BINS
        for angle, dist_m, intensity in pkt.points:
            if not (RANGE_MIN_M <= dist_m <= RANGE_MAX_M):
                continue
            b = ld_to_ros_bin(angle)
            if math.isinf(self._ranges[b]) or dist_m < self._ranges[b]:
                self._ranges[b] = dist_m
                self._intensities[b] = float(intensity)
        self._last_start = start
        self._speed_dps = pkt.speed_dps
        return scan


def main(argv=None) -> None:
    import rclpy
    import serial
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=argv)
    node = rclpy.create_node("ld06_lidar")
    node.declare_parameter("port", DEFAULT_PORT)
    node.declare_parameter("baud", DEFAULT_BAUD)
    node.declare_parameter("frame_id", DEFAULT_FRAME_ID)
    port = node.get_parameter("port").value
    baud = node.get_parameter("baud").value
    frame_id = node.get_parameter("frame_id").value

    pub = node.create_publisher(LaserScan, "/scan", 10)
    # 打不开端口直接抛异常退出(串口独占,多实例立刻暴露)
    ser = serial.Serial(port, baud, timeout=0.1)
    node.get_logger().info(f"LD06 opened {port} @ {baud}, publishing /scan")

    parser = Ld06Parser()
    assembler = ScanAssembler()

    def publish(ranges: list[float], intensities: list[float], speed_dps: float) -> None:
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / BINS
        msg.angle_max = msg.angle_increment * (BINS - 1)
        # 转速(度/秒)-> 一圈用时;丢包时按上一圈转速近似
        msg.scan_time = 360.0 / speed_dps if speed_dps > 0 else 0.0
        msg.time_increment = msg.scan_time / BINS
        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M
        msg.ranges = ranges
        msg.intensities = intensities
        pub.publish(msg)

    import threading

    stop = threading.Event()

    def reader() -> None:
        while not stop.is_set():
            data = ser.read(512)
            if not data:
                continue
            for pkt in parser.feed(data):
                scan = assembler.add_packet(pkt)
                if scan is not None:
                    publish(*scan)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1.0)
        ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
