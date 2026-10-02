from abc import ABC, abstractmethod

DEADZONE = 0.02  # 占空比死区:小于该值视为停止,避免微占空比下电机嗡鸣


def normalize_duty(duty):
    """占空比归一化:截断到 [-1, 1],死区内归零。所有平台统一走这里。"""
    duty = max(-1.0, min(1.0, float(duty)))
    return duty if abs(duty) > DEADZONE else 0.0


class Drivetrain(ABC):
    """底盘驱动接口,所有平台(raspberrypi / mock / 未来移植)统一实现。

    本层只做忠实执行:接收左右轮占空比 [-1, 1],正=向前转,负=向后转。
    vx/wz → 左右轮的差速混合不在此层,由 core/motion 负责。

    子类只需实现 _write_motors() 与 close();set_speeds()/stop() 由基类
    提供,截断与死区统一在此处理,保证各平台行为一致。

    last_command 记录最近一次归一化后的指令,mock 编码器据此仿真车轮转动。
    """

    last_command = (0.0, 0.0)

    def set_speeds(self, left: float, right: float) -> None:
        """设置左右轮占空比(截断到 [-1,1],死区内归零)"""
        self.last_command = (normalize_duty(left), normalize_duty(right))
        self._write_motors(*self.last_command)

    @abstractmethod
    def _write_motors(self, left: float, right: float) -> None:
        """把已归一化的左右轮占空比写入电机。子类实现此方法,不要覆盖 set_speeds。"""

    def stop(self) -> None:
        """双轮停止(滑行)"""
        self.set_speeds(0.0, 0.0)

    @abstractmethod
    def close(self) -> None:
        """停车并释放 GPIO 资源;程序退出(含异常)时必须被调用"""

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()


class Encoders(ABC):
    """编码器接口,与 Drivetrain 同层的平台抽象(M5 里程计)。

    read() 返回左右轮累计有符号 tick 数,正 = 车轮向"前进"方向旋转;
    A/B 相接反、左右装反等接线问题在各平台实现内用符号常数修正,
    上层(core/odometry)只认本约定。
    """

    @abstractmethod
    def read(self) -> tuple:
        """读取左右轮累计 tick 数 (left, right),int,可正可负"""

    @abstractmethod
    def reset(self) -> None:
        """计数清零(里程计本身另有基线,互不影响)"""

    @abstractmethod
    def close(self) -> None:
        """释放 GPIO 资源;必须幂等,允许 finally / with 中重复调用"""

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
