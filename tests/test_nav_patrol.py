"""tools/nav_patrol 测试:航点 JSON 校验、位姿转换、巡逻主循环(假发送端)。"""
import json
import math

import pytest

from src.tools.nav_patrol import load_waypoints, run_patrol, waypoints_to_pose_data

GOOD = {"frame": "map", "loop": True,
        "waypoints": [{"x": 1.2, "y": 0.6, "theta": 0.0}, {"x": 3.0, "y": 2.1, "theta": 1.57}]}


def _write(tmp_path, data):
    path = tmp_path / "wp.json"
    path.write_text(data if isinstance(data, str) else json.dumps(data), encoding="utf-8")
    return path


def test_load_waypoints_valid(tmp_path):
    data = load_waypoints(_write(tmp_path, GOOD))
    assert data["loop"] is True
    assert data["waypoints"] == [(1.2, 0.6, 0.0), (3.0, 2.1, 1.57)]


@pytest.mark.parametrize(
    "data",
    [
        GOOD | {"frame": "odom"},  # 坐标系不符
        GOOD | {"loop": 1},  # loop 非 bool
        GOOD | {"waypoints": []},  # 空列表
        GOOD | {"waypoints": [{"x": 1.0, "y": 0.6}]},  # 缺 theta
        GOOD | {"waypoints": [{"x": True, "y": 0.6, "theta": 0.0}]},  # bool 不是数值
        GOOD | {"waypoints": [{"x": "1.0", "y": 0.6, "theta": 0.0}]},  # 字符串
        GOOD | {"waypoints": [{"x": float("nan"), "y": 0.6, "theta": 0.0}]},  # nan
        GOOD | {"waypoints": [{"x": float("inf"), "y": 0.6, "theta": 0.0}]},  # inf
        "not json",
        "[1, 2]",
    ],
)
def test_load_waypoints_rejects_invalid(tmp_path, data):
    with pytest.raises(ValueError):
        load_waypoints(_write(tmp_path, data))


def test_load_waypoints_missing_file(tmp_path):
    with pytest.raises(ValueError):
        load_waypoints(tmp_path / "nope.json")


def test_waypoints_to_pose_data_quaternion():
    poses = waypoints_to_pose_data([(1.0, 2.0, math.pi / 2)])
    assert poses[0]["x"] == 1.0
    assert poses[0]["y"] == 2.0
    assert poses[0]["qz"] == pytest.approx(math.sin(math.pi / 4))
    assert poses[0]["qw"] == pytest.approx(math.cos(math.pi / 4))


def test_run_patrol_single_pass():
    sent = []
    data = {"loop": False, "waypoints": [(1.0, 2.0, 0.0), (3.0, 4.0, 1.0)]}
    missed = run_patrol(lambda poses: sent.append(poses) or [1], data, log_fn=lambda _: None)
    assert len(sent) == 1  # 不循环
    assert len(sent[0]) == 2
    assert missed == [1]


def test_run_patrol_loops_until_interrupted():
    calls = []

    def send(poses):
        calls.append(poses)
        if len(calls) == 2:
            raise KeyboardInterrupt
        return []

    data = {"loop": True, "waypoints": [(1.0, 2.0, 0.0)]}
    logs = []
    with pytest.raises(KeyboardInterrupt):
        run_patrol(send, data, log_fn=logs.append)
    assert len(calls) == 2
    assert any("全部到达" in line for line in logs)
