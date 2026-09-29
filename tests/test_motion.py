"""core/motion 测试:差速混合公式、输入截断、80% 占空比上限。"""
import pytest

from src.core.motion import MAX_DUTY, mix


def test_straight_forward():
    assert mix(0.5, 0.0) == (0.5, 0.5)


def test_straight_backward():
    assert mix(-0.5, 0.0) == (-0.5, -0.5)


def test_spin_left():
    # wz>0 左转:左轮后退、右轮前进,等值反向
    left, right = mix(0.0, 0.4)
    assert left == pytest.approx(-0.4)
    assert right == pytest.approx(0.4)


def test_forward_with_left_turn():
    # vx=0.6, wz=0.2:左轮 0.4,右轮 0.8(右快→向左)
    assert mix(0.6, 0.2) == (pytest.approx(0.4), pytest.approx(0.8))


def test_zero():
    assert mix(0.0, 0.0) == (0.0, 0.0)


def test_input_clamped():
    assert mix(5.0, 0.0) == (MAX_DUTY, MAX_DUTY)


def test_overflow_normalized_keeping_ratio():
    # vx=1, wz=0.5 → 原始 (0.5, 1.5),等比缩小为 (1/3, 1),再受 80% 上限约束
    left, right = mix(1.0, 0.5)
    assert left == pytest.approx(1 / 3 * MAX_DUTY)
    assert right == pytest.approx(MAX_DUTY)
    assert right == pytest.approx(3 * left)  # 比例保持 1:3


def test_duty_never_exceeds_max():
    for vx10 in range(-10, 11):
        for wz10 in range(-10, 11):
            left, right = mix(vx10 / 10, wz10 / 10)
            assert -MAX_DUTY <= left <= MAX_DUTY
            assert -MAX_DUTY <= right <= MAX_DUTY
