"""平台层入口:接口见 base.py,工厂按运行环境选择具体平台实现。

移植新平台:在本目录新建子文件夹实现 base.Drivetrain / base.Encoders,
并在 _REGISTRY / _ENCODERS_REGISTRY 注册一行(详见《软件架构设计.md》第 7 节)。
"""
import importlib
import logging
import os

from .base import Drivetrain, Encoders

log = logging.getLogger(__name__)

_PLATFORM_ENV = "ROVER_PLATFORM"

_REGISTRY = {
    "mock": "src.platforms.mock.drivetrain:MockDrivetrain",
    "raspberrypi": "src.platforms.raspberrypi.drivetrain:RaspberryPiDrivetrain",
}

_ENCODERS_REGISTRY = {
    "mock": "src.platforms.mock.encoders:MockEncoders",
    "raspberrypi": "src.platforms.raspberrypi.encoders:RaspberryPiEncoders",
}


def _load(spec):
    module_name, class_name = spec.rsplit(":", 1)
    return getattr(importlib.import_module(module_name), class_name)


def create_drivetrain():
    """创建当前平台的底盘实例。

    环境变量 ROVER_PLATFORM=mock|raspberrypi 可强制指定;
    缺省自动探测:GPIO 可用 → raspberrypi,否则 → mock 仿真。
    """
    name = os.environ.get(_PLATFORM_ENV, "").strip().lower()
    if name:
        if name not in _REGISTRY:
            raise ValueError(f"未知平台 {name!r},可选:{sorted(_REGISTRY)}")
        log.info("platform: %s(环境变量指定)", name)
        return _load(_REGISTRY[name])()
    try:
        from .raspberrypi.drivetrain import RaspberryPiDrivetrain

        drivetrain = RaspberryPiDrivetrain()
    except Exception as exc:
        log.warning("GPIO 不可用(%s),回退到 mock 仿真平台", exc)
        from .mock.drivetrain import MockDrivetrain

        return MockDrivetrain()
    log.info("platform: raspberrypi(自动探测)")
    return drivetrain


def _build_encoders(name, drivetrain):
    """按平台名实例化编码器;mock 需要底盘引用以仿真车轮转动。"""
    cls = _load(_ENCODERS_REGISTRY[name])
    return cls(drivetrain) if name == "mock" else cls()


def create_encoders(drivetrain=None):
    """创建当前平台的编码器实例,平台选择语义与 create_drivetrain 一致。

    drivetrain 仅 mock 平台使用(按指令积分);不传则 mock 静止不动。
    """
    name = os.environ.get(_PLATFORM_ENV, "").strip().lower()
    if name:
        if name not in _ENCODERS_REGISTRY:
            raise ValueError(f"未知平台 {name!r},可选:{sorted(_ENCODERS_REGISTRY)}")
        log.info("encoders platform: %s(环境变量指定)", name)
        return _build_encoders(name, drivetrain)
    try:
        from .raspberrypi.encoders import RaspberryPiEncoders

        encoders = RaspberryPiEncoders()
    except Exception as exc:
        log.warning("GPIO 不可用(%s),回退到 mock 仿真编码器", exc)
        return _build_encoders("mock", drivetrain)
    log.info("encoders platform: raspberrypi(自动探测)")
    return encoders


__all__ = ["Drivetrain", "Encoders", "create_drivetrain", "create_encoders"]
