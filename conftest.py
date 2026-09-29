"""pytest 全局配置:raspberrypi 平台测试使用 gpiozero 虚拟引脚,不碰真实 GPIO。

MockFactory 默认引脚类 MockPin 不支持 PWM,必须显式指定 MockPWMPin。
"""
from gpiozero import Device
from gpiozero.pins.mock import MockFactory, MockPWMPin

Device.pin_factory = MockFactory(pin_class=MockPWMPin)
