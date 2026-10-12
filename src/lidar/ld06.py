"""LD06 激光雷达协议解析。

纯 Python 实现,不 import serial/rclpy,离线环境可完整单测。
协议(LD06 开发手册):串口 230400 8N1 单向输出,47 字节定长包:

    0      1        2~3     4~5        6~41           42~43    44~45    46
    0x54   0x2C     speed   start      12 x 测距点    end      ts       crc8
    帧头   帧型+点数  度/秒   起始角     (距离mm 2B     结束角   毫秒
                             (0.01°)    +强度 1B)    (0.01°)

角度约定:0° 朝雷达正前方,顺时针增大;一包内 12 点角度按起止角线性插值。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

HEADER_BYTE = 0x54
VERLEN_BYTE = 0x2C  # 高 3 位帧型=1,低 5 位测距点数=12
PACKET_LEN = 47
POINTS_PER_PACKET = 12

RANGE_MIN_M = 0.02
RANGE_MAX_M = 12.0


def _build_crc_table() -> list[int]:
    # CRC-8,多项式 0x4D,MSB 先行,初值 0x00(LD06 手册查表法的生成式)
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x4D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC_TABLE = _build_crc_table()


def crc8(data: bytes | bytearray) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


@dataclass
class Ld06Packet:
    speed_dps: float  # 转速,度/秒
    timestamp_ms: int
    # (角度 deg, 距离 m, 强度);角度为雷达系:正前方 0°,顺时针,已归一到 [0,360)
    points: list[tuple[float, float, int]] = field(default_factory=list)


def _parse_packet(raw: bytes) -> Ld06Packet:
    speed = struct.unpack_from("<H", raw, 2)[0]
    start = struct.unpack_from("<H", raw, 4)[0] / 100.0
    end = struct.unpack_from("<H", raw, 42)[0] / 100.0
    ts = struct.unpack_from("<H", raw, 44)[0]
    span = end - start
    if span < 0:
        span += 360.0  # 一包跨过 360° 回绕
    points = []
    for i in range(POINTS_PER_PACKET):
        off = 6 + i * 3
        dist_m = struct.unpack_from("<H", raw, off)[0] / 1000.0
        intensity = raw[off + 2]
        angle = (start + span * i / (POINTS_PER_PACKET - 1)) % 360.0
        points.append((angle, dist_m, intensity))
    return Ld06Packet(speed_dps=float(speed), timestamp_ms=ts, points=points)


class Ld06Parser:
    """增量式解析器:feed() 接受任意切片字节流,返回校验通过的完整包列表。

    同步策略:搜 0x54 0x2C 帧头 -> 凑满 47B -> CRC8 校验;
    校验失败丢 1 字节重新同步,保证坏包/错位后自动恢复。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Ld06Packet]:
        self._buf.extend(data)
        packets = []
        while True:
            pkt = self._try_pop()
            if pkt is None:
                return packets
            packets.append(pkt)

    def _try_pop(self) -> Ld06Packet | None:
        buf = self._buf
        while True:
            # 找帧头候选:0x54 后跟 0x2C;末尾半个 0x54 保留等下个字节
            i = buf.find(bytes([HEADER_BYTE]))
            while i != -1 and i + 1 < len(buf) and buf[i + 1] != VERLEN_BYTE:
                i = buf.find(bytes([HEADER_BYTE]), i + 1)
            if i == -1:
                buf.clear()
                return None
            if i + 1 >= len(buf):
                if i:
                    del buf[:i]
                return None
            if i:
                del buf[:i]
            if len(buf) < PACKET_LEN:
                return None
            raw = bytes(buf[:PACKET_LEN])
            if crc8(raw[:-1]) == raw[-1]:
                del buf[:PACKET_LEN]
                return _parse_packet(raw)
            del buf[0]  # CRC 失败:丢 1 字节重新同步
