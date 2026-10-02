"""仿真编码器:按底盘最近一次指令对时间积分,免硬件跑通里程计(FR-5)。

懒积分:不在后台线程持续累加,而是 read() 时用注入时钟算 dt,
按 last_command 占空比 × 标称轮速 × dt 换算 tick;测试也可用
advance() 直接注入精确 tick 数。
"""
import math
import time

from ..base import Encoders

TICKS_PER_REV = 1560  # 13 线 × 减速比 30 × 4 倍频,与真机一致
WHEEL_DIAMETER_M = 0.065  # 占位值,待 M5 真机标定
DEFAULT_TICKS_PER_METER = TICKS_PER_REV / (math.pi * WHEEL_DIAMETER_M)  # ≈7643
DEFAULT_MAX_WHEEL_SPEED = 0.5  # 占空比 1.0 时的标称轮速 m/s,占位值


class _StationaryDrivetrain:
    """未关联底盘时的哑替身:指令恒为停止。"""

    last_command = (0.0, 0.0)


class MockEncoders(Encoders):
    def __init__(
        self,
        drivetrain=None,
        ticks_per_meter=DEFAULT_TICKS_PER_METER,
        max_wheel_speed=DEFAULT_MAX_WHEEL_SPEED,
        clock=time.monotonic,
    ):
        self._drivetrain = drivetrain or _StationaryDrivetrain()
        self._ticks_per_meter = ticks_per_meter
        self._max_wheel_speed = max_wheel_speed
        self._clock = clock
        self._ticks = [0.0, 0.0]  # 内部用 float 累计,read() 取整,避免丢小数
        self._last_time = None

    def read(self):
        now = self._clock()
        if self._last_time is None:  # 首次调用只建立时间基线
            self._last_time = now
            return (int(self._ticks[0]), int(self._ticks[1]))
        dt = now - self._last_time
        self._last_time = now
        for i, duty in enumerate(self._drivetrain.last_command):
            self._ticks[i] += duty * self._max_wheel_speed * dt * self._ticks_per_meter
        return (int(self._ticks[0]), int(self._ticks[1]))

    def advance(self, left_ticks, right_ticks):
        """测试专用:直接注入精确 tick 增量。"""
        self._ticks[0] += left_ticks
        self._ticks[1] += right_ticks

    def reset(self):
        self._ticks = [0.0, 0.0]

    def close(self):  # 幂等:无资源可释放
        pass
