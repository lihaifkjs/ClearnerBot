"""LD06 协议解析器与 scan 组装的离线单测(无需硬件/ROS)。"""

import math
import struct

from src.lidar.ld06 import (
    PACKET_LEN,
    POINTS_PER_PACKET,
    Ld06Parser,
    crc8,
)
from src.lidar.ld06_node import BINS, ScanAssembler, ld_to_ros_bin

# LD06 手册样例包(真机抓包前先用它做 CRC 回归):
# speed=2152°/s, start=324.27°, end=334.70°, ts=6714ms, crc=0x50
SAMPLE_PACKET = bytes([
    0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E,
    0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00, 0xE5,
    0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9,
    0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2, 0xC7, 0x00, 0xE9,
    0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5,
    0xBE, 0x82, 0x3A, 0x1A, 0x50,
])


def build_packet(speed_dps, start_cdeg, points, end_cdeg, ts_ms):
    """构造一包合法数据(CRC 自动补齐)。points: 12 x (dist_mm, intensity)"""
    assert len(points) == POINTS_PER_PACKET
    raw = bytearray()
    raw += struct.pack("<BBHH", 0x54, 0x2C, speed_dps, start_cdeg)
    for dist_mm, intensity in points:
        raw += struct.pack("<HB", dist_mm, intensity)
    raw += struct.pack("<HH", end_cdeg, ts_ms)
    raw.append(crc8(raw))
    assert len(raw) == PACKET_LEN
    return bytes(raw)


def simple_points(dist_mm=1000, intensity=100):
    return [(dist_mm, intensity)] * POINTS_PER_PACKET


# ---------- CRC 与单包解析 ----------


def test_sample_packet_crc_passes():
    assert crc8(SAMPLE_PACKET[:-1]) == SAMPLE_PACKET[-1]


def test_sample_packet_fields():
    (pkt,) = Ld06Parser().feed(SAMPLE_PACKET)
    assert pkt.speed_dps == 2152
    assert pkt.timestamp_ms == 6714
    first_angle, first_dist, first_inten = pkt.points[0]
    assert first_angle == 324.27
    assert first_dist == 0.224
    assert first_inten == 0xE4
    # 末点角度 = 结束角,距离 192mm
    last_angle, last_dist, _ = pkt.points[-1]
    assert last_angle == 334.70
    assert last_dist == 0.192
    # 角度线性递增
    angles = [p[0] for p in pkt.points]
    assert angles == sorted(angles)


def test_angle_interpolation_wraps_360():
    pkt_data = build_packet(3600, 35000, simple_points(), 1000, 0)  # 350° -> 10°
    (pkt,) = Ld06Parser().feed(pkt_data)
    angles = [p[0] for p in pkt.points]
    assert angles[0] == 350.0
    assert angles[-1] == 10.0
    # 插值单调跨过 360°:350,351.8,...,359.1,0.9,...,10
    assert angles[5] > 359.0
    assert 0.0 < angles[6] < 1.5


# ---------- 增量式喂入与同步 ----------


def test_feed_byte_by_byte_equals_whole():
    stream = (
        build_packet(3600, 0, simple_points(1000), 3000, 1)
        + build_packet(3600, 3000, simple_points(2000), 6000, 2)
        + build_packet(3600, 6000, simple_points(3000), 9000, 3)
    )
    whole = Ld06Parser().feed(stream)
    drip = Ld06Parser()
    dripped = []
    for b in stream:
        dripped += drip.feed(bytes([b]))
    assert len(whole) == len(dripped) == 3
    assert [p.timestamp_ms for p in dripped] == [1, 2, 3]
    assert dripped[1].points[0][1] == 2.0


def test_garbage_prefix_skipped():
    stream = b"\x00\xff\x54\x99garbage" + build_packet(
        3600, 0, simple_points(), 3000, 7
    )
    (pkt,) = Ld06Parser().feed(stream)
    assert pkt.timestamp_ms == 7


def test_bad_crc_dropped_and_resyncs():
    good1 = build_packet(3600, 0, simple_points(), 3000, 1)
    bad = bytearray(build_packet(3600, 3000, simple_points(), 6000, 2))
    bad[10] ^= 0xFF  # 破坏 CRC
    good2 = build_packet(3600, 6000, simple_points(), 9000, 3)
    pkts = Ld06Parser().feed(good1 + bytes(bad) + good2)
    assert [p.timestamp_ms for p in pkts] == [1, 3]


def test_truncated_packet_waits_for_more():
    parser = Ld06Parser()
    full = build_packet(3600, 0, simple_points(), 3000, 5)
    assert parser.feed(full[:20]) == []
    (pkt,) = parser.feed(full[20:])
    assert pkt.timestamp_ms == 5


# ---------- scan 组装 ----------


def test_ld_to_ros_bin_direction():
    assert ld_to_ros_bin(0.0) == 0  # 正前方 -> bin 0
    assert ld_to_ros_bin(90.0) == 270  # 雷达顺时针 90°(右方) -> ROS 逆时针 270°
    assert ld_to_ros_bin(270.0) == 90
    assert ld_to_ros_bin(359.5) == 0 or ld_to_ros_bin(359.5) == 1


def test_assembler_emits_full_scan_on_wrap():
    asm = ScanAssembler()
    packets = [
        build_packet(3600, a, simple_points(1000 + a), a + 2900, i)
        for i, a in enumerate(range(0, 36000, 3000))
    ]
    packets.append(build_packet(3600, 0, simple_points(1000), 2900, 12))
    scans = []
    for p in packets:
        (pkt,) = Ld06Parser().feed(p)
        scan = asm.add_packet(pkt)
        if scan:
            scans.append(scan)
    # 12 包铺满一圈(0°~359°),第 13 包起始角回落到 0° -> 触发首圈发射
    assert len(scans) == 1
    ranges, intensities, speed = scans[0]
    assert len(ranges) == BINS == len(intensities)
    assert speed == 3600
    assert any(not math.isinf(r) for r in ranges)


def test_assembler_invalid_points_stay_inf():
    asm = ScanAssembler()
    invalid = [(0, 50)] * POINTS_PER_PACKET  # 距离 0 = 无效
    # 三圈数据全无效:第 3 包起始角回落触发发射,圈内应全 inf
    stream = (
        build_packet(3600, 0, invalid, 3000, 1)
        + build_packet(3600, 3000, invalid, 6000, 2)
        + build_packet(3600, 0, invalid, 3000, 3)
    )
    scan = None
    parser = Ld06Parser()
    for pkt in parser.feed(stream):
        scan = asm.add_packet(pkt) or scan
    assert scan is not None
    ranges, _, _ = scan
    assert all(math.isinf(r) for r in ranges)


def test_assembler_keeps_nearest_in_bin():
    asm = ScanAssembler()
    # 同一角度给两个距离,bin 内应保留更近的
    near = build_packet(3600, 10000, simple_points(800), 12900, 1)
    far = build_packet(3600, 10000, simple_points(1500), 12900, 2)
    wrap = build_packet(3600, 0, simple_points(500), 1000, 3)
    parser = Ld06Parser()
    scan = None
    for pkt in parser.feed(near + far + wrap):
        scan = asm.add_packet(pkt) or scan
    ranges, _, _ = scan
    assert ranges[ld_to_ros_bin(100.0)] == 0.8
