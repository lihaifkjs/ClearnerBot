"""sim/raycast 测试:PGM/yaml 解析、射线投射几何(全部离线纯函数)。"""
import math

import pytest

from src.sim.raycast import SimMap, cast_ray, load_map, make_scan

STEP_TOL = 0.51  # 步进采样沿射线方向的误差 ≤ 步长(resolution/2 = 0.5)


def _border_map(size=10, resolution=1.0):
    """size×size、四周一圈墙的地图,原点 (0,0)。row 0 = 北边(y 大)。"""
    grid = [[False] * size for _ in range(size)]
    for i in range(size):
        grid[0][i] = True  # 北墙
        grid[size - 1][i] = True  # 南墙
        grid[i][0] = True  # 西墙
        grid[i][size - 1] = True  # 东墙
    return SimMap(grid=grid, resolution=resolution, origin_x=0.0, origin_y=0.0)


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 4.0),  # 向东:x 从 5 到东墙 x=9
        (math.pi / 2, 4.0),  # 向北:y 从 5 到北墙 y=9
        (-math.pi / 2, 4.5),  # 向南:y 从 5 到南墙 y<1
        (math.pi, 4.5),  # 向西:x 从 5 到西墙 x<1
        (math.pi / 4, 4 * math.sqrt(2)),  # 东北角:x 与 y 同时到 9
    ],
)
def test_cast_ray_directions(angle, expected):
    sim_map = _border_map()
    assert cast_ray(sim_map, 5.0, 5.0, angle) == pytest.approx(expected, abs=STEP_TOL)


def test_cast_ray_empty_map_returns_max_range():
    sim_map = SimMap(grid=[[False] * 10 for _ in range(10)],
                     resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert cast_ray(sim_map, 5.0, 5.0, 0.0, max_range=8.0) == 8.0


def test_cast_ray_next_to_wall():
    sim_map = _border_map()
    assert cast_ray(sim_map, 8.6, 5.0, 0.0) == pytest.approx(0.4, abs=STEP_TOL)


def test_make_scan_beam_count_and_symmetry():
    sim_map = _border_map()
    ranges = make_scan(sim_map, 5.0, 5.0, 0.0, beams=360)
    assert len(ranges) == 360
    # 东(0°)与北(90°)墙距离相等(方形房间中心)
    assert ranges[0] == pytest.approx(ranges[90])


def _write_map(tmp_path, pixels, width, height, negate=0):
    """写最小 pgm(4x2 级)+ yaml;pixels 按行展开,0=黑(占用)、255=白(空闲)。"""
    pgm = tmp_path / "t.pgm"
    pgm.write_bytes(b"P5\n# comment\n" + f"{width} {height}\n255\n".encode() + bytes(pixels))
    yaml = tmp_path / "t.yaml"
    yaml.write_text(
        f"image: t.pgm\nresolution: 0.5\norigin: [1.0, 2.0, 0]\n"
        f"negate: {negate}\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    return yaml


def test_load_map_parsing_and_occupancy(tmp_path):
    # 4x2:首行两端黑(墙),其余白
    yaml = _write_map(tmp_path, [0, 255, 255, 0, 255, 255, 255, 255], 4, 2)
    sim_map = load_map(yaml)
    assert (sim_map.width, sim_map.height) == (4, 2)
    assert (sim_map.resolution, sim_map.origin_x, sim_map.origin_y) == (0.5, 1.0, 2.0)
    assert sim_map.grid[0] == [True, False, False, True]  # pgm 首行 = row 0
    assert sim_map.grid[1] == [False] * 4
    # row 0(北边)对应世界 y = origin_y + (height-1)*resolution = 2.5
    assert sim_map.is_occupied(1.1, 2.6) is True
    assert sim_map.is_occupied(1.1, 2.1) is False


def test_load_map_negate_flips_occupancy(tmp_path):
    yaml = _write_map(tmp_path, [0, 255, 255, 0, 255, 255, 255, 255], 4, 2, negate=1)
    sim_map = load_map(yaml)
    assert sim_map.grid[0] == [False, True, True, False]


def test_load_map_rejects_bad_pgm(tmp_path):
    pgm = tmp_path / "t.pgm"
    pgm.write_bytes(b"P2\n1 1\n255\n0\n")  # ASCII PGM 不支持
    yaml = tmp_path / "t.yaml"
    yaml.write_text("image: t.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_map(yaml)
