import logging

from gpiozero import DigitalOutputDevice, PWMOutputDevice

from ..base import Drivetrain
from . import pins

log = logging.getLogger(__name__)

PWM_FREQUENCY = 1000  # Hz;人耳对低频 PWM 啸叫敏感,1kHz 较安静


class RaspberryPiDrivetrain(Drivetrain):
    def __init__(self):
        self._closed = False
        self._standby = DigitalOutputDevice(pins.STBY)
        self._pwm_l = PWMOutputDevice(pins.PWMA, frequency=PWM_FREQUENCY)
        self._in1_l = DigitalOutputDevice(pins.AIN1)
        self._in2_l = DigitalOutputDevice(pins.AIN2)
        self._pwm_r = PWMOutputDevice(pins.PWMB, frequency=PWM_FREQUENCY)
        self._in1_r = DigitalOutputDevice(pins.BIN1)
        self._in2_r = DigitalOutputDevice(pins.BIN2)
        self._standby.on()
        log.info("TB6612 已使能(STBY=GPIO%d)", pins.STBY)

    @staticmethod
    def _set_motor(pwm, in1, in2, duty):
        # TB6612 真值表:IN1=0,IN2=1 正转;IN1=1,IN2=0 反转;同 0 滑行停止
        # duty 已经 base 归一化(截断+死区),此处直接判断符号
        if duty > 0:
            in1.off()
            in2.on()
            pwm.value = duty
        elif duty < 0:
            in1.on()
            in2.off()
            pwm.value = -duty
        else:
            in1.off()
            in2.off()
            pwm.value = 0.0

    def _write_motors(self, left, right):
        self._set_motor(self._pwm_l, self._in1_l, self._in2_l, left)
        self._set_motor(self._pwm_r, self._in1_r, self._in2_r, right)

    def close(self):
        if self._closed:  # 幂等:允许 finally / with 中重复调用
            return
        self._closed = True
        self.stop()
        self._standby.off()  # 拉低使能,切断电机供电,防止退出后电机保持状态
        for dev in (
            self._pwm_l,
            self._in1_l,
            self._in2_l,
            self._pwm_r,
            self._in1_r,
            self._in2_r,
            self._standby,
        ):
            dev.close()
