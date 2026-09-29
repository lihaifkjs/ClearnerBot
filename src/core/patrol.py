"""往返巡逻状态机(开环):前进半个周期 → 后退半个周期,循环往复。

周期为时间近似,受电机差异与地面打滑影响,本期不要求精确(PRD FR-2)。
手动遥控优先:巡逻中收到 drive 指令即退出巡逻(架构 §4)。

时钟注入:clock 默认 time.monotonic,测试时传入假时钟拨动时间,不真实 sleep。
"""
import time

from .motion import mix

MIN_SPEED, MAX_SPEED = 0.1, 1.0  # 巡逻速度取值范围(协议 v1)
MIN_PERIOD, MAX_PERIOD = 1.0, 30.0  # 往返周期取值范围,秒(协议 v1)
DEFAULT_SPEED = 0.5
DEFAULT_PERIOD = 4.0


def _clamp(value, lo, hi):
    return max(lo, min(hi, float(value)))


class Patrol:
    def __init__(self, drivetrain, clock=time.monotonic):
        self._drivetrain = drivetrain
        self._clock = clock
        self.active = False
        self.speed = DEFAULT_SPEED
        self.period = DEFAULT_PERIOD
        self._started_at = 0.0

    def start(self, speed=None, period=None):
        """开启巡逻;缺省参数沿用上次设置,越界值截断到协议范围。"""
        if speed is not None:
            self.speed = _clamp(speed, MIN_SPEED, MAX_SPEED)
        if period is not None:
            self.period = _clamp(period, MIN_PERIOD, MAX_PERIOD)
        self.active = True
        self._started_at = self._clock()
        self._apply()

    def stop(self):
        if self.active:
            self.active = False
            self._drivetrain.stop()

    def handle_manual_drive(self):
        """手动优先:巡逻中收到手动 drive 指令,自动退出巡逻。"""
        self.stop()

    def tick(self):
        """由外层控制循环周期性调用;按已流逝时间决定当前处于前进段还是后退段。"""
        if self.active:
            self._apply()

    def _apply(self):
        phase = (self._clock() - self._started_at) % self.period
        direction = 1.0 if phase < self.period / 2 else -1.0
        left, right = mix(direction * self.speed, 0.0)
        self._drivetrain.set_speeds(left, right)
