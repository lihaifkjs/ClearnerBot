"""编码器测试:raspberrypi 实现走 gpiozero 虚拟引脚,mock 实现走假时钟。

上拉输入:引脚低电平 = 设备逻辑 1。正转序列 00→01→11→10→00
(B 拉低、A 拉低、B 拉高、A 拉高)每整周期 4 个沿 = +4 tick。
"""
import math

import pytest
from gpiozero import Device

from src.platforms.mock.encoders import MockEncoders
from src.platforms.raspberrypi import encoders as rpi_enc
from src.platforms.raspberrypi.encoders import RaspberryPiEncoders
from src.platforms.raspberrypi import pins

ENC_PINS = (pins.ENC_LEFT_A, pins.ENC_LEFT_B, pins.ENC_RIGHT_A, pins.ENC_RIGHT_B)


def _pin(n):
    return Device.pin_factory.pin(n)


def _drive_cycle(a_pin, b_pin, reverse=False):
    """驱动一整周期正交序列;mock 引脚的边沿回调在 drive_* 调用内同步触发。"""
    a, b = _pin(a_pin), _pin(b_pin)
    steps = (
        [a.drive_low, b.drive_low, a.drive_high, b.drive_high]
        if reverse
        else [b.drive_low, a.drive_low, b.drive_high, a.drive_high]
    )
    for step in steps:
        step()


@pytest.fixture
def encoders():
    for n in ENC_PINS:  # 先建立已知电平(0,0),再构造,保证周期计数精确
        _pin(n).drive_high()
    enc = RaspberryPiEncoders()
    yield enc
    enc.close()


def test_forward_cycle_counts_4x(encoders):
    _drive_cycle(pins.ENC_LEFT_A, pins.ENC_LEFT_B)
    assert encoders.read() == (4, 0)
    _drive_cycle(pins.ENC_RIGHT_A, pins.ENC_RIGHT_B)
    assert encoders.read() == (4, 4)
    _drive_cycle(pins.ENC_LEFT_A, pins.ENC_LEFT_B)  # 每周期恰好 +4
    assert encoders.read() == (8, 4)


def test_reverse_cycle_counts_negative(encoders):
    _drive_cycle(pins.ENC_LEFT_A, pins.ENC_LEFT_B, reverse=True)
    assert encoders.read() == (-4, 0)


def test_bounce_nets_zero(encoders):
    # 单相来回抖动:去程 +1、回程 -1,非法/抖动转移净计数为 0
    b = _pin(pins.ENC_LEFT_B)
    for _ in range(5):
        b.drive_low()
        b.drive_high()
    assert encoders.read() == (0, 0)


def test_sign_constants_fix_wiring(monkeypatch):
    # A/B 相接反只改符号常数:LEFT_SIGN=-1 后正转序列计为负
    monkeypatch.setattr(rpi_enc, "LEFT_SIGN", -1)
    for n in ENC_PINS:  # 建立已知电平后再构造,保证周期计数精确
        _pin(n).drive_high()
    enc = RaspberryPiEncoders()
    try:
        _drive_cycle(pins.ENC_LEFT_A, pins.ENC_LEFT_B)
        assert enc.read() == (-4, 0)
        _drive_cycle(pins.ENC_RIGHT_A, pins.ENC_RIGHT_B)  # 右轮不受影响
        assert enc.read() == (-4, 4)
    finally:
        enc.close()


def test_reset(encoders):
    _drive_cycle(pins.ENC_LEFT_A, pins.ENC_LEFT_B)
    encoders.reset()
    assert encoders.read() == (0, 0)


def test_close_idempotent(encoders):
    encoders.close()
    encoders.close()


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class StubDrivetrain:
    def __init__(self):
        self.last_command = (0.0, 0.0)


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def mock_encoders(clock):
    return MockEncoders(StubDrivetrain(), ticks_per_meter=1000.0,
                        max_wheel_speed=0.5, clock=clock)


def test_mock_integrates_duty_over_time(mock_encoders, clock):
    enc = mock_encoders
    assert enc.read() == (0, 0)  # 首次调用只建立时间基线
    enc._drivetrain.last_command = (1.0, 1.0)
    clock.advance(2.0)  # 0.5 m/s × 2 s × 1000 tick/m = 1000 tick
    assert enc.read() == (1000, 1000)
    clock.advance(1.0)  # 继续累计
    assert enc.read() == (1500, 1500)


def test_mock_negative_duty_negative_ticks(mock_encoders, clock):
    enc = mock_encoders
    enc.read()
    enc._drivetrain.last_command = (-1.0, -0.5)
    clock.advance(1.0)
    left, right = enc.read()
    assert left == -500
    assert right == -250


def test_mock_stopped_when_no_drivetrain(clock):
    enc = MockEncoders(clock=clock)  # 不传底盘 → 静止
    enc.read()
    clock.advance(10.0)
    assert enc.read() == (0, 0)


def test_mock_advance_injects_exact_ticks(mock_encoders):
    mock_encoders.advance(120, -30)
    assert mock_encoders.read() == (120, -30)


def test_mock_reset(mock_encoders):
    mock_encoders.advance(120, -30)
    mock_encoders.reset()
    assert mock_encoders.read() == (0, 0)


def test_mock_default_ticks_per_meter():
    enc = MockEncoders()
    assert enc._ticks_per_meter == pytest.approx(1560 / (math.pi * 0.065))


def test_mock_close_idempotent(mock_encoders):
    mock_encoders.close()
    mock_encoders.close()
