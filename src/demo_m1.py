"""M1 接线验证 demo:前进 → 后退 → 原地自旋 → 停止。

真机:轮子悬空后在项目根目录运行 `python3 -m src.demo_m1`;
其他电脑:自动回退 mock 仿真平台,指令打印到终端(先仿真后实物)。
"""
import logging
import time

from src.platforms import create_drivetrain

SPEED = 0.4  # 低速验证,防止意外冲出

# (说明, (左轮, 右轮), 持续秒数);原地自旋 = 左右轮等速反向
STEPS = [
    ("前进", (SPEED, SPEED), 2.0),
    ("后退", (-SPEED, -SPEED), 2.0),
    ("原地自旋", (-SPEED, SPEED), 1.5),
]


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        with create_drivetrain() as drivetrain:
            for label, (left, right), seconds in STEPS:
                print(label)
                drivetrain.set_speeds(left, right)
                time.sleep(seconds)
            print("停止")
    except KeyboardInterrupt:
        print("已中断,电机停止")


if __name__ == "__main__":
    main()
