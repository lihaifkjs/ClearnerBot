# ClearnerBot

基于树莓派 4B 的 WiFi 遥控小车:手机浏览器(H5)遥控 + 激光雷达 SLAM 建图 + Nav2 航点巡逻。

- 硬件:Raspberry Pi 4B(8GB)+ TT 电机(带霍尔编码器)+ 乐动 LD06 激光雷达
- 软件:Python 3 车端服务(FastAPI/WebSocket)+ ROS2 Humble(RoboStack)+ slam_toolbox + Nav2
- 详情见根目录各阶段文档(先读 [PRD.md](PRD.md))

## 目录结构

```
src/
├── core/        # 业务逻辑:运动混合、巡逻、看门狗、里程计(平台无关)
├── platforms/   # 平台抽象:raspberrypi(GPIO/编码器)与 mock(无硬件仿真)
├── comm/        # 通信:WebSocket/HTTP 服务、协议校验、ROS2 /cmd_vel 桥接
├── sim/         # 仿真:对地图射线投射产假 /scan(M8)
├── tools/       # 工具:里程计标定、航点巡逻 CLI
└── main.py      # 入口:组装三层
web/index.html   # H5 遥控页
config/          # robot.json(速度上限/雷达 TF)、slam_toolbox.yaml、航点样例
maps/            # M7 建图产出(pgm/yaml/png)
tests/           # 离线单测,全部跑在 mock 平台上
```

## 开发(Mac/无硬件)

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q        # 全量离线测试(mock 平台)
.venv/bin/python -m src.main          # 仿真模式启动(终端打印电机指令)
```

ROS 相关模块(rclpy/tf2/nav2_msgs)全部延迟导入,无 ROS 环境不影响测试。

## 部署与运行(树莓派)

ROS2 环境为 RoboStack Humble(`~/miniforge3/envs/ros`),代码 rsync 到 `~/ClearnerBot`:

```bash
# 主服务(电机/编码器/H5/ROS2 桥接,自动探测平台)
~/miniforge3/envs/ros/bin/python -m src.main

# 手机遥控:浏览器打开 http://<pi-ip>:8000
```

SLAM 建图、导航启动链等运维细节见《M7 建图.md》《M8 导航巡逻.md》。

## 里程碑进度

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M1~M3 | 底盘点亮、电机控制、H5 遥控 + 往返巡逻 | ✅ |
| M4 | ROS2 环境与 /cmd_vel 桥接 | ✅ |
| M5 | 编码器里程计(轮径/轮距标定) | ✅ |
| M6 | LD06 雷达接入(/scan) | ✅ |
| M7 | slam_toolbox 建图 | ✅ |
| M8 | Nav2 航点巡逻(含动态避障) | 🔧 进行中 |
| M9 | 手机地图界面(标点巡逻) | 未开始 |

## 文档索引

- [PRD.md](PRD.md) — 产品需求(含 M4~M9 路线)
- [软件架构设计.md](软件架构设计.md) — 分层结构与通信协议
- [硬件选型与BOM.md](硬件选型与BOM.md) — 硬件清单与接线
- [M2 电机控制.md](M2%20电机控制.md) / [M3手机遥控.md](M3手机遥控.md) / [M4 ROS2桥接.md](M4%20ROS2桥接.md) / [M5 编码器里程计.md](M5%20编码器里程计.md) / [M6 雷达接入.md](M6%20雷达接入.md) / [M7 建图.md](M7%20建图.md) / [M8 导航巡逻.md](M8%20导航巡逻.md) — 各阶段方案、验收记录与排障备忘
- [树莓派连接与环境配置.md](树莓派连接与环境配置.md) — SSH 免密与网络优化
