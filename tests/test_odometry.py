"""core/odometry 里程计测试:假时钟拨动时间,不真实 sleep。"""
import math

import pytest

from src.core.odometry import Odometry, OdometryParams

TICKS_PER_REV = 1560  # 与硬件定值一致
WHEEL_D = 0.065
TRACK = 0.15


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
def odo(clock):
    o = Odometry(OdometryParams(), clock=clock)
    o.update(0, 0)  # 建立 tick 基线
    return o


def test_straight_line_one_rev(odo):
    odo.update(TICKS_PER_REV, TICKS_PER_REV)  # 双轮各转一圈
    assert odo.pose == (pytest.approx(math.pi * WHEEL_D), pytest.approx(0.0),
                        pytest.approx(0.0))


def test_spin_in_place(odo):
    half = TICKS_PER_REV // 2  # 左轮倒转半圈、右轮正转半圈 → 原地旋转
    odo.update(-half, half)
    x, y, theta = odo.pose
    expected = (2 * half * math.pi * WHEEL_D / TICKS_PER_REV) / TRACK
    assert theta == pytest.approx(expected)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_arc_midpoint_integration(odo):
    # 右轮走 2 倍左轮距离:恒定曲率圆弧;dtheta 精确,弦长 ≈ dc(小角度)
    dl_ticks, dr_ticks = 1560, 3120
    odo.update(dl_ticks, dr_ticks)
    mpt = math.pi * WHEEL_D / TICKS_PER_REV
    dc = (dl_ticks + dr_ticks) * mpt / 2
    dtheta = (dr_ticks - dl_ticks) * mpt / TRACK
    x, y, theta = odo.pose
    assert theta == pytest.approx(dtheta)
    assert math.hypot(x, y) == pytest.approx(dc, rel=1e-3)  # 弦长 ≈ 弧长
    assert x > 0 and y > 0  # 左轮慢 → 向左转,y(左)为正


def test_velocity_from_clock(odo, clock):
    clock.advance(0.5)
    odo.update(TICKS_PER_REV, TICKS_PER_REV)
    expected_vx = math.pi * WHEEL_D / 0.5
    assert odo.velocity == (pytest.approx(expected_vx), pytest.approx(0.0))


def test_zero_dt_keeps_pose_updates_velocity(odo, clock):
    odo.update(100, 100)
    vx_before = odo.vx
    # dt=0:位姿照常积分,速度不更新、不崩溃
    x_before = odo.x
    odo.update(200, 200)
    assert odo.x > x_before
    assert odo.vx == vx_before


def test_reset_rebaselines_no_pose_jump(odo):
    odo.update(1000, 1000)
    assert odo.x > 0
    odo.reset()
    assert odo.pose == (0.0, 0.0, 0.0)
    assert odo.velocity == (0.0, 0.0)
    # 编码器计数仍在走:reset 后首次 update 只重建基线,不产生位姿跳变
    odo.update(5000, 5000)
    assert odo.pose == (0.0, 0.0, 0.0)


def test_params_save_load_roundtrip(tmp_path):
    params = OdometryParams(ticks_per_rev=1560, wheel_diameter_m=0.067,
                            track_width_m=0.142)
    path = tmp_path / "calibration.json"
    params.save(path)
    assert OdometryParams.load(path) == params


def test_params_missing_file_defaults(tmp_path):
    assert OdometryParams.load(tmp_path / "nope.json") == OdometryParams()


def test_params_extra_keys_ignored(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"wheel_diameter_m": 0.07, "junk": 1}')
    params = OdometryParams.load(path)
    assert params.wheel_diameter_m == 0.07
    assert params.track_width_m == TRACK


def test_params_invalid_json_defaults(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text("not json")
    assert OdometryParams.load(path) == OdometryParams()
