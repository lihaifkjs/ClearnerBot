"""core/safety 看门狗测试:假时钟拨动时间,不真实 sleep。"""
import pytest

from src.core.safety import Watchdog
from src.platforms.mock.drivetrain import MockDrivetrain


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def drivetrain():
    return MockDrivetrain()


@pytest.fixture
def watchdog(drivetrain, clock):
    return Watchdog(drivetrain, timeout=0.5, clock=clock)


def test_no_timeout_no_stop(watchdog, drivetrain, clock):
    clock.advance(0.4)
    assert watchdog.check() is False
    assert drivetrain.calls == []


def test_timeout_parks_once(watchdog, drivetrain, clock):
    clock.advance(0.6)
    assert watchdog.check() is True
    assert drivetrain.calls == [(0.0, 0.0)]
    assert watchdog.check() is False  # 已触发,不重复停车
    assert drivetrain.calls == [(0.0, 0.0)]


def test_feed_refreshes_timer(watchdog, drivetrain, clock):
    clock.advance(0.4)
    watchdog.feed()
    clock.advance(0.4)
    assert watchdog.check() is False
    assert drivetrain.calls == []


def test_feed_after_expiry_recovers(watchdog, drivetrain, clock):
    clock.advance(0.6)
    watchdog.check()
    watchdog.feed()
    clock.advance(0.4)
    assert watchdog.check() is False


def test_continuous_feeding_never_triggers(watchdog, drivetrain, clock):
    # 模拟巡逻/按住摇杆:外层循环持续喂狗,看门狗不触发
    for _ in range(20):
        clock.advance(0.1)
        watchdog.feed()
        assert watchdog.check() is False
    assert drivetrain.calls == []
