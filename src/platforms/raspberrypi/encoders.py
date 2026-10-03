"""树莓派霍尔编码器读取:4x 正交解码,纯 GPIO 边沿计数(无需额外库)。

TT 电机编码器 13 线 × 减速比 30 × 4 倍频 = 每轮每转 1560 tick。
A/B 相 3.3V 直驱 GPIO,上拉输入;左右轮各两个沿(上升+下降)全部计数。
"""
import threading

from gpiozero import DigitalInputDevice

from ..base import Encoders
from . import pins

# 接线修正:A/B 相接反或左右装反时,只改这两个符号,不动接线
# (先例:TB6612 左右轮通道对调在 drivetrain.py 内修正)
# 2026-10-03 真机实测:左轮向前转为正(+);右轮向前转为负,故 RIGHT_SIGN 取 -1
LEFT_SIGN = +1
RIGHT_SIGN = -1

# 4x 正交解码状态转移表:索引 (prev << 2) | cur,prev/cur 为 (A<<1)|B 的 2bit 状态。
# 正转序列 00→01→11→10→00,反转反之;每整周期 4 个沿 = 4 tick(4 倍频)。
# 非法跳变(双边同变,多为抖动/丢沿)与原地不变记 0,天然滤抖。
_TRANSITIONS = {
    0b0001: +1, 0b0111: +1, 0b1110: +1, 0b1000: +1,
    0b0010: -1, 0b1011: -1, 0b1101: -1, 0b0100: -1,
}


class _WheelCounter:
    """单轮正交计数器:记录上一状态,任意沿触发时按转移表累加。"""

    def __init__(self, dev_a, dev_b, sign, lock):
        self._dev_a = dev_a
        self._dev_b = dev_b
        self._sign = sign
        self._lock = lock
        self._prev = self._state()
        self.count = 0

    def _state(self):
        return (int(self._dev_a.value) << 1) | int(self._dev_b.value)

    def on_edge(self):
        cur = self._state()
        delta = _TRANSITIONS.get((self._prev << 2) | cur, 0) * self._sign
        self._prev = cur
        with self._lock:
            self.count += delta


class RaspberryPiEncoders(Encoders):
    def __init__(
        self,
        left_a=pins.ENC_LEFT_A,
        left_b=pins.ENC_LEFT_B,
        right_a=pins.ENC_RIGHT_A,
        right_b=pins.ENC_RIGHT_B,
    ):
        self._closed = False
        self._lock = threading.Lock()  # gpiozero 回调在后台线程触发,计数需加锁
        self._devs = [
            DigitalInputDevice(pin, pull_up=True)
            for pin in (left_a, left_b, right_a, right_b)
        ]
        self._left = _WheelCounter(self._devs[0], self._devs[1], LEFT_SIGN, self._lock)
        self._right = _WheelCounter(self._devs[2], self._devs[3], RIGHT_SIGN, self._lock)
        # 每轮 A/B 两相的上升沿、下降沿都触发解码,实现 4 倍频
        for dev, wheel in (
            (self._devs[0], self._left),
            (self._devs[1], self._left),
            (self._devs[2], self._right),
            (self._devs[3], self._right),
        ):
            dev.when_activated = wheel.on_edge
            dev.when_deactivated = wheel.on_edge

    def read(self):
        with self._lock:
            return (self._left.count, self._right.count)

    def reset(self):
        with self._lock:
            self._left.count = 0
            self._right.count = 0

    def close(self):
        if self._closed:  # 幂等:允许 finally / with 中重复调用
            return
        self._closed = True
        for dev in self._devs:
            dev.close()
