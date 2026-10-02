"""差速底盘里程计:编码器 tick → 位姿 (x, y, theta) 与速度 (vx, wz)。

坐标约定:x 向前、y 向左、theta 逆时针为正(与 ROS 一致)。
轮径/轮距为占位默认值,真机用 src/tools/calibrate_odometry.py 标定,
结果存 calibration.json,由 OdometryParams.load 读取。
"""
import json
import math
import time
from dataclasses import asdict, dataclass, fields
from pathlib import Path

TICKS_PER_REV = 1560  # 13 线 × 减速比 30 × 4 倍频(硬件定值,非占位)


@dataclass
class OdometryParams:
    ticks_per_rev: int = TICKS_PER_REV
    wheel_diameter_m: float = 0.065  # 占位值,待真机标定
    track_width_m: float = 0.15  # 占位值,待真机标定

    @classmethod
    def load(cls, path):
        """读取标定文件;文件缺失/损坏 → 占位默认值,多余字段忽略。"""
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return cls()
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})

    def save(self, path):
        Path(path).write_text(
            json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


class Odometry:
    """增量式里程计:每次 update 用 tick 差分推一个微小弧段并累加。

    差速运动学(圆弧模型):左右轮在 dt 内各走 dl/dr,底盘中心走
    dc = (dl+dr)/2,航向变化 dtheta = (dr-dl)/L(L=轮距)。用中点航向
    theta_mid = theta + dtheta/2 积分(二阶精度),直线与原地旋转均精确,
    恒定曲率圆弧下误差为 O(dtheta²),50ms 节拍下可忽略。
    """

    def __init__(self, params, clock=time.monotonic):
        self.params = params
        self._clock = clock
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self._last_ticks = None
        self._last_time = None

    def update(self, left_ticks, right_ticks):
        now = self._clock()
        if self._last_ticks is None:  # 首次(或 reset 后)只建立基线
            self._last_ticks = (left_ticks, right_ticks)
            self._last_time = now
            return
        dl_ticks = left_ticks - self._last_ticks[0]
        dr_ticks = right_ticks - self._last_ticks[1]
        self._last_ticks = (left_ticks, right_ticks)

        meters_per_tick = math.pi * self.params.wheel_diameter_m / self.params.ticks_per_rev
        dl = dl_ticks * meters_per_tick
        dr = dr_ticks * meters_per_tick
        dc = (dl + dr) / 2
        dtheta = (dr - dl) / self.params.track_width_m

        theta_mid = self.theta + dtheta / 2
        self.x += dc * math.cos(theta_mid)
        self.y += dc * math.sin(theta_mid)
        self.theta += dtheta

        dt = now - self._last_time
        self._last_time = now
        if dt > 0:  # dt<=0(假时钟未拨动等)只积分位姿,速度保持上次值
            self.vx = dc / dt
            self.wz = dtheta / dt

    @property
    def pose(self):
        return (self.x, self.y, self.theta)

    @property
    def velocity(self):
        return (self.vx, self.wz)

    def as_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "vx": self.vx,
            "wz": self.wz,
        }

    def reset(self):
        """位姿/速度清零;tick 基线置空,下次 update 重新对齐编码器计数。"""
        self.x = self.y = self.theta = 0.0
        self.vx = self.wz = 0.0
        self._last_ticks = None
        self._last_time = None
