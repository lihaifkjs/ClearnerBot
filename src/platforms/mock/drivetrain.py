import logging

from ..base import Drivetrain

log = logging.getLogger(__name__)


class MockDrivetrain(Drivetrain):
    """仿真平台:电机指令打印到终端;calls 记录全部调用序列,供单元测试断言。"""

    def __init__(self):
        self.calls = []

    def set_speeds(self, left, right):
        state = (round(float(left), 3), round(float(right), 3))
        if not self.calls or self.calls[-1] != state:
            log.info("motor left=%.2f right=%.2f", *state)
        self.calls.append(state)

    def stop(self):
        self.set_speeds(0.0, 0.0)

    def close(self):
        self.stop()
