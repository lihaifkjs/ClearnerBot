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
    """

    def set_speeds(self, left: float, right: float) -> None:
        """设置左右轮占空比(截断到 [-1,1],死区内归零)"""
        self._write_motors(normalize_duty(left), normalize_duty(right))

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
