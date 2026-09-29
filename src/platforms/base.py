from abc import ABC, abstractmethod


class Drivetrain(ABC):
    """底盘驱动接口,所有平台(raspberrypi / mock / 未来移植)统一实现。

    本层只做忠实执行:接收左右轮占空比 [-1, 1],正=向前转,负=向后转。
    vx/wz → 左右轮的差速混合不在此层,由 core/motion 负责。
    """

    @abstractmethod
    def set_speeds(self, left: float, right: float) -> None:
        """设置左右轮占空比,取值 [-1, 1]"""

    @abstractmethod
    def stop(self) -> None:
        """双轮停止(滑行)"""

    @abstractmethod
    def close(self) -> None:
        """停车并释放 GPIO 资源;程序退出(含异常)时必须被调用"""

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
