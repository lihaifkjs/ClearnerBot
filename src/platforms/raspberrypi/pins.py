"""TB6612 ↔ 树莓派 4B 引脚映射(BCM 编号),对应《硬件选型与BOM.md》第 4 节。"""

PWMA = 12  # 物理 32,A 通道 PWM(实物接右轮,见 drivetrain.py 注释)
AIN1 = 17  # 物理 11,A 通道方向
AIN2 = 27  # 物理 13,A 通道方向
PWMB = 13  # 物理 33,B 通道 PWM(实物接左轮)
BIN1 = 22  # 物理 15,B 通道方向
BIN2 = 23  # 物理 16,B 通道方向
STBY = 24  # 物理 18,驱动板使能(高=工作,低=断电)

# 霍尔编码器 A/B 相(3.3V 直驱 GPIO,上拉输入),对应《硬件选型与BOM.md》
ENC_LEFT_A = 5  # 物理 29,左轮 A 相
ENC_LEFT_B = 6  # 物理 31,左轮 B 相
ENC_RIGHT_A = 19  # 物理 35,右轮 A 相
ENC_RIGHT_B = 26  # 物理 37,右轮 B 相
