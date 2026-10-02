"""里程计交互式标定(真实硬件):测轮径与轮距,写入 calibration.json。

用法(树莓派上):python3 -m src.tools.calibrate_odometry
  第 1 步:推车直行 1.0 m → 轮径
  第 2 步:原地旋转整数圈 → 轮距
mock 平台无真实编码数据,直接提示退出。
"""
import math
from pathlib import Path

from src.core.odometry import OdometryParams
from src.platforms import create_encoders

CALIBRATION_PATH = Path(__file__).resolve().parents[2] / "calibration.json"
PUSH_DISTANCE_M = 1.0  # 第 1 步推车距离
DEFAULT_TURNS = 3  # 第 2 步建议旋转圈数,圈数越多量化误差越小


def _delta(before, after):
    return after[0] - before[0], after[1] - before[1]


def _calibrate_wheel_diameter(encoders, params):
    input("第 1 步:把车对准起点标记,回车后沿直线推车 "
          f"{PUSH_DISTANCE_M} m,到位后再回车。\n就绪后按回车开始...")
    before = encoders.read()
    input(f"推车 {PUSH_DISTANCE_M} m 到位后按回车...")
    dl_ticks, dr_ticks = _delta(before, encoders.read())

    # tick = 距离 / (π·D) × 每转 tick 数 → D = 距离 × 每转 tick 数 / (π·tick)
    diameters = []
    for name, ticks in (("左轮", dl_ticks), ("右轮", dr_ticks)):
        if ticks <= 0:
            raise SystemExit(f"{name} tick 增量为 {ticks},请沿前进方向推车后重试")
        d = PUSH_DISTANCE_M * params.ticks_per_rev / (math.pi * ticks)
        diameters.append(d)
        print(f"{name}: {ticks} tick → 轮径 {d * 1000:.2f} mm")
    if abs(diameters[0] - diameters[1]) / max(diameters) > 0.05:
        print("警告:左右轮径相差超过 5%,请检查推车是否走直线、轮胎是否打滑")
    return sum(diameters) / 2


def _calibrate_track_width(encoders, params):
    answer = input(f"第 2 步:原地旋转整数圈后回到起始朝向,建议 {DEFAULT_TURNS} 圈。\n"
                   f"计划转几圈?[{DEFAULT_TURNS}] ").strip()
    turns = int(answer) if answer else DEFAULT_TURNS
    input("就绪后按回车开始...")
    before = encoders.read()
    input(f"原地转满 {turns} 圈、回到起始朝向后按回车...")
    dl_ticks, dr_ticks = _delta(before, encoders.read())

    # 原地旋转:两轮等距反向,各自行走圆弧长 = π·L·N(L=轮距,N=圈数),
    # 故轮距 L = (dr - dl) / (2π·N),dr/dl 为轮行走距离(带符号)
    meters_per_tick = math.pi * params.wheel_diameter_m / params.ticks_per_rev
    dr = dr_ticks * meters_per_tick
    dl = dl_ticks * meters_per_tick
    track = abs(dr - dl) / (2 * math.pi * turns)
    if track <= 0:
        raise SystemExit("两轮 tick 无差异,请确认是原地旋转而非平移")
    print(f"左轮 {dl_ticks} tick / 右轮 {dr_ticks} tick → 轮距 {track * 1000:.1f} mm")
    return track


def main():
    params = OdometryParams.load(CALIBRATION_PATH)
    print(f"标定前参数:{params}")
    encoders = create_encoders()
    try:
        if "mock" in type(encoders).__module__:
            raise SystemExit("当前为 mock 仿真平台,无真实编码器数据,请在树莓派上运行")
        params.wheel_diameter_m = _calibrate_wheel_diameter(encoders, params)
        params.track_width_m = _calibrate_track_width(encoders, params)
    finally:
        encoders.close()  # 异常退出也要释放 GPIO
    params.save(CALIBRATION_PATH)
    print(f"标定后参数:{params}\n已写入 {CALIBRATION_PATH}")


if __name__ == "__main__":
    main()
