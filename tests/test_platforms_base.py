"""base 统一行为测试:截断与死区对所有平台生效(经 mock 平台验证)。"""
from src.platforms.base import DEADZONE, normalize_duty
from src.platforms.mock.drivetrain import MockDrivetrain


def test_normalize_clamps_to_unit():
    assert normalize_duty(5.0) == 1.0
    assert normalize_duty(-5.0) == -1.0


def test_normalize_deadzone():
    assert normalize_duty(DEADZONE / 2) == 0.0
    assert normalize_duty(-DEADZONE / 2) == 0.0
    assert normalize_duty(DEADZONE * 2) == DEADZONE * 2


def test_mock_receives_normalized_values():
    # 死区内的指令经 base 归一化后,mock 记录到的应为停止
    dt = MockDrivetrain()
    dt.set_speeds(0.01, -0.01)
    assert dt.calls[-1] == (0.0, 0.0)


def test_mock_receives_clamped_values():
    dt = MockDrivetrain()
    dt.set_speeds(5.0, -5.0)
    assert dt.calls[-1] == (1.0, -1.0)


def test_stop_default_implementation():
    # stop() 由 base 提供,等价于 set_speeds(0, 0)
    dt = MockDrivetrain()
    dt.set_speeds(0.5, 0.5)
    dt.stop()
    assert dt.calls[-1] == (0.0, 0.0)
