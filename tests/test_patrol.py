"""core/patrol、core/safety 共用的假时钟:测试里拨动时间,不真实 sleep。"""
import pytest

from src.core.patrol import DEFAULT_PERIOD, MAX_PERIOD, MAX_SPEED, MIN_PERIOD, MIN_SPEED, Patrol
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
def patrol(drivetrain, clock):
    return Patrol(drivetrain, clock=clock)


def last_call(drivetrain):
    return drivetrain.calls[-1]


def test_start_drives_forward_at_speed(patrol, drivetrain):
    patrol.start(speed=0.5)
    assert patrol.active
    assert last_call(drivetrain) == (0.5, 0.5)


def test_default_params_when_not_given(patrol):
    patrol.start()
    assert patrol.speed == 0.5
    assert patrol.period == DEFAULT_PERIOD


def test_params_clamped_to_protocol_range(patrol):
    patrol.start(speed=5.0, period=999.0)
    assert patrol.speed == MAX_SPEED
    assert patrol.period == MAX_PERIOD
    patrol.start(speed=0.0, period=0.0)
    assert patrol.speed == MIN_SPEED
    assert patrol.period == MIN_PERIOD


def test_flips_to_backward_after_half_period(patrol, drivetrain, clock):
    patrol.start(speed=0.4, period=4.0)
    clock.advance(2.1)
    patrol.tick()
    assert last_call(drivetrain) == (-0.4, -0.4)


def test_back_to_forward_after_full_period(patrol, drivetrain, clock):
    patrol.start(speed=0.4, period=4.0)
    clock.advance(4.1)
    patrol.tick()
    assert last_call(drivetrain) == (0.4, 0.4)


def test_speed_capped_by_max_duty(patrol, drivetrain):
    patrol.start(speed=1.0)
    assert last_call(drivetrain) == (0.8, 0.8)  # core/motion 的 80% 占空比上限


def test_manual_drive_exits_patrol(patrol, drivetrain):
    patrol.start()
    patrol.handle_manual_drive()
    assert not patrol.active
    assert last_call(drivetrain) == (0.0, 0.0)


def test_stop_parks_and_is_idempotent(patrol, drivetrain):
    patrol.stop()  # 未启动时调用不报错、不写电机
    assert drivetrain.calls == []
    patrol.start()
    calls_before = len(drivetrain.calls)
    patrol.stop()
    patrol.stop()
    assert not patrol.active
    assert len(drivetrain.calls) == calls_before + 1
    assert last_call(drivetrain) == (0.0, 0.0)


def test_tick_noop_when_inactive(patrol, drivetrain, clock):
    clock.advance(10.0)
    patrol.tick()
    assert drivetrain.calls == []
