# M4 ROS2 环境与 /cmd_vel 桥接 — 技术方案

- 日期:2026-10-04
- 状态:**已完成,真机验收通过**(2026-10-04)
- 里程碑 tag:`v0.4-ros2-bridge`
- 关联:[PRD.md](PRD.md) §10.3(M4 行)、[M3手机遥控.md](M3手机遥控.md)、[M5 编码器里程计.md](M5%20编码器里程计.md)

## 1. 目标与契约(PRD §10.3)

- 内容:ROS2 安装、工作空间、`/cmd_vel` 桥接节点(翻译成现有运动控制 API)
- 产出契约:`/cmd_vel`(geometry_msgs/Twist,线速度 m/s + 角速度 rad/s)
- 独立验证:命令行 `ros2 topic pub /cmd_vel ...`,小车动(先仿真后真机)

## 2. 已确认的关键决策

| 决策点 | 结论 | 理由 |
|---|---|---|
| ROS2 安装方式 | RoboStack(miniforge + mamba 装 ros-base) | Pi OS 非 Ubuntu 无官方 apt 包;不重装系统、免编译、aarch64 支持成熟 |
| ROS 发行版 | Humble(LTS 到 2027) | RoboStack aarch64 生态最熟;后续 M7/M8 的 slam_toolbox/Nav2 包齐全 |
| 桥接形态 | 嵌入现有 `src.main` 进程 | rclpy 后台线程 spin,直接调现有 drive 入口;GPIO 单实例拥有;复用看门狗 |
| 速度参数配置 | `config/robot.json`(入库) | 与机器相关的 `calibration.json` 区分:标定值不入库,方案参数入库 |

## 3. 设计要点

- **单 Python 环境**:嵌入形态要求 rclpy 与现有代码同解释器 → 整个车端服务改在
  RoboStack 环境里跑,fastapi/uvicorn/websockets/gpiozero/lgpio 均装入 conda env
- **模块位置**:`src/comm/ros_bridge.py`(桥接属通信层);rclpy 延迟导入,
  Mac 离线环境(无 ROS)现有测试不受影响;Twist→(vx,wz) 换算做成纯函数,可单测
- **单位换算**:`/cmd_vel` 物理单位 → `mix(vx, wz)` 归一化 [-1,1]:
  `vx = linear.x / max_linear_speed`,`wz = angular.z / max_angular_speed`,
  越界由现有 `mix` 的 clamp/峰值等比缩小兜底
- **速度参数初值**:`max_linear_speed` 0.5 m/s(M5 mock 标称轮速占位);
  `max_angular_speed` 按轮距 141.7mm 推算 ≈ 7 rad/s;真机实测后修正
- **安全联动**:收到 `/cmd_vel` 即喂看门狗 → 指令流中断 0.5s 自动停车,
  与 FR-3 同路径(`src/core/safety.py`),Nav2 持续发令场景天然兼容
- **部署变化**:树莓派新增 miniforge;服务启动命令从 `.venv/bin/python` 换为
  conda env 的 python;systemd 自启(FR-4 欠账)做部署时一并补上

## 4. 风险与第一步验证

- **风险**:lgpio 是 C 扩展,gpiozero 经 lgpio 驱动 GPIO;需验证在 conda env
  (自带 Python,非系统 3.11.2)里能否正常编译/工作
- **第一步(实施顺序 1)**:树莓派上装 miniforge → 建 Humble 环境 →
  验证 `import rclpy`、`import gpiozero`(lgpio 后端)都可用;
  不通则退回"桥接独立进程 + 走现有 WS 协议当第二客户端"方案
- 实施顺序:① 环境可行性验证 → ② `config/robot.json` + `ros_bridge.py`
  (换算纯函数 + 节点) → ③ main.py 嵌入接线 → ④ 仿真验证 → ⑤ 真机验证

### 4.1 环境可行性验证记录(2026-10-04,真机)

全部通过,嵌入方案成立:

1. miniforge 装至 `~/miniforge3`(conda 26.7.2)
2. `mamba create -n ros -c conda-forge -c robostack-staging ros-humble-ros-base
   python=3.11` 成功
3. `import rclpy` + geometry_msgs/Twist 读写 ✓;rclpy 节点 init/spin ✓
4. pip 装 gpiozero 2.0.1 / lgpio 0.2.2(C 扩展编译成功)/ fastapi / uvicorn /
   websockets / httpx ✓
5. **conda env 内 lgpio 实测打开 /dev/gpiochip0 成功**(GPIO 访问不受 conda 影响)✓
6. 当前代码(89 条测试)在 conda env 全绿 ✓;**直接调 env 的 python
   即可用 rclpy/ros2,无需 conda activate**(RoboStack 包用 RPATH)

坑位备忘:

- **pytest 必须 <9**:ros 环境自带 launch_testing 插件与 pytest 9 的
  hookspec 不兼容(报 PluginValidationError),已 pin `pytest<9`
- 树莓派 `~/ClearnerBot` 是 rsync 部署、非 git 仓库;本次顺手把代码从
  M3 旧版同步到最新(M5),`calibration.json` 因无 --delete 未受影响
- 原 `.venv` 启动方式作废,服务改用 `~/miniforge3/envs/ros/bin/python`
  启动(部署步骤 ③ 时落实)

### 4.2 真机联调排障记录(2026-10-04)

首次真机"前行 1m"失败,症状与定位链:

1. 车物理直行远超 1m 才停,但 odom 显示"原地右转 9.7 圈、x 不涨"
   → 左轮有 tick、右轮 tick 恒 0 → 右编码器链路故障
2. 手转右轮,tick 在 0/1 间抖动 → 正交解码单相缺信号的指纹(A/B 缺一相)
3. 逐引脚测边沿:GPIO26 有 2391 跳变,GPIO19 零跳变且上拉下仍被拽低
4. 同一线挪到物理 36(BCM16)复测:3686 跳变 → **树莓派 BCM19 输入级损坏**,
   线与编码器无故障;修复 = `pins.py` 右轮 A 相改 GPIO16(线留物理 36)

教训(后续里程碑注意):

- **停止条件不能盲信里程计**:本轮停止条件是"odom.x≥1.0",编码器故障导致
  条件永不满足,靠 15s 超时兜底;以后真机动作指令应以"理论时长 + 上限"
  做主停止,odom 只做观测
- 树莓派单个 GPIO 输入级会坏(原因不明,可能静电/热插拔);诊断手法
  "信号线挪到空闲脚复测"可快速区分线/传感器故障与引脚故障

## 5. 验证方法

- 离线:换算纯函数单测(单位换算、越界 clamp、零值);现有 89 条测试保持全绿
  (rclpy 缺失时桥接不启动,服务行为不变)
- 仿真:mock 平台启动服务,`ros2 topic pub /cmd_vel` 终端打印电机指令;
  停发 0.5s 后看门狗停车
- 真机:同仿真操作,小车按指令运动;松手(停发)即停

## 6. 交接说明 / 后续

### 6.1 真机验收记录(2026-10-04)

- 环境:RoboStack Humble(`~/miniforge3/envs/ros`),服务改由该环境 python 启动
- 仿真(mock 平台 + 真 ROS2):`ros2 topic pub /cmd_vel`(0.25 m/s, 3.5 rad/s)
  → mock 打印 `left=0.00 right=0.80`(换算/mix/峰值缩放正确);停发 0.5s
  看门狗停车 ✓
- 真机:① 发 `linear.x=0.25` 至 odom.x≥1.0 → 实测前进 ≈1.3m(odom 1.32,
  轮径标定很准);② 发 `angular.z=-3.0` 至 Δθ≥3 圈 → 实测右转 ≈4.5 圈
  (odom 3.5 圈,轮距标定值偏大约 29%)
- **验收结论(PRD §10.3 M4):命令行发 /cmd_vel,小车动 ✓;停发自动停车 ✓**

### 6.2 遗留与后续

- **精标定(欠账,用户确认缓做)**:轮距实际 ≈ 标定值/1.29;`config/robot.json`
  的 max_linear/angular_speed 仍是占位初值 0.5/7.0,M8 导航前需按实测车速修正
  (真机动作按"理论时长 + 上限"控制,见 §4.2 教训)
- **systemd 常驻自启(FR-4 欠账)**:做部署时补,启动命令换为
  `~/miniforge3/envs/ros/bin/python -m src.main`
- M5→/odom 发布:桥接层已落地,把 `odometry.as_dict()` 映射到
  nav_msgs/Odometry 即可(M5 交接已预留,数据源就绪;不在 M4 契约内)
- M6(雷达):LD06 驱动可直接在本 conda 环境装 ROS 驱动包
