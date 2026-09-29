"""platforms/raspberrypi 引脚级测试。

gpiozero MockFactory 提供虚拟引脚,无需真实硬件即可断言
每个 GPIO 的电平/PWM 占空比(对应 TB6612 真值表)。
"""
import pytest
from gpiozero import Device

from src.platforms.raspberrypi import pins
from src.platforms.raspberrypi.drivetrain import RaspberryPiDrivetrain


def pin_state(pin):
    return Device.pin_factory.pin(pin).state


@pytest.fixture
def drivetrain():
    dt = RaspberryPiDrivetrain()
    yield dt
    dt.close()


def test_standby_enabled_on_init(drivetrain):
    assert pin_state(pins.STBY) == 1


def test_left_forward(drivetrain):
    # 正占空比:AIN1=0, AIN2=1, PWMA=占空比(TB6612 真值表)
    drivetrain.set_speeds(0.5, 0.0)
    assert pin_state(pins.AIN1) == 0
    assert pin_state(pins.AIN2) == 1
    assert pin_state(pins.PWMA) == pytest.approx(0.5)


def test_left_backward(drivetrain):
    # 负占空比:IN 引脚对调
    drivetrain.set_speeds(-0.5, 0.0)
    assert pin_state(pins.AIN1) == 1
    assert pin_state(pins.AIN2) == 0
    assert pin_state(pins.PWMA) == pytest.approx(0.5)


def test_spin_in_place(drivetrain):
    # 原地自旋:左右轮等速反向
    drivetrain.set_speeds(-0.4, 0.4)
    assert pin_state(pins.AIN1) == 1
    assert pin_state(pins.AIN2) == 0
    assert pin_state(pins.BIN1) == 0
    assert pin_state(pins.BIN2) == 1


def test_deadzone_treated_as_stop(drivetrain):
    drivetrain.set_speeds(0.01, 0.01)
    assert pin_state(pins.PWMA) == 0
    assert pin_state(pins.PWMB) == 0


def test_out_of_range_clamped(drivetrain):
    drivetrain.set_speeds(5.0, -5.0)
    assert pin_state(pins.PWMA) == pytest.approx(1.0)
    assert pin_state(pins.PWMB) == pytest.approx(1.0)


def test_stop_clears_all(drivetrain):
    drivetrain.set_speeds(0.8, 0.8)
    drivetrain.stop()
    for pin in (pins.PWMA, pins.AIN1, pins.AIN2, pins.PWMB, pins.BIN1, pins.BIN2):
        assert pin_state(pin) == 0


def test_close_disables_standby(drivetrain):
    drivetrain.close()
    assert pin_state(pins.STBY) == 0


def test_close_idempotent(drivetrain):
    drivetrain.close()
    drivetrain.close()
