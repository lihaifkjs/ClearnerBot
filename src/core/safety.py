"""看门狗:超过 timeout 未收到指令则自动停车(FR-3 断连保护)。

只负责"指令流中断停车";WebSocket 断开的停车由 comm 层在连接关闭时直接处理。
巡逻期间电机在执行有意指令,由外层控制循环持续喂狗,不会触发。

时钟注入:clock 默认 time.monotonic,测试时传入假时钟拨动时间,不真实 sleep。
"""
import time


class Watchdog:
    def __init__(self, drivetrain, timeout=0.5, clock=time.monotonic):
        self._drivetrain = drivetrain
        self.timeout = timeout
        self._clock = clock
        self._last_feed = clock()
        self._expired = False  # 触发后只停车一次,避免重复写电机

    def feed(self):
        """收到有效指令时调用,刷新计时。"""
        self._last_feed = self._clock()
        self._expired = False

    def check(self):
        """由外层控制循环周期性调用;超时则停车并返回 True(仅触发当次)。"""
        if not self._expired and self._clock() - self._last_feed > self.timeout:
            self._expired = True
            self._drivetrain.stop()
            return True
        return False
