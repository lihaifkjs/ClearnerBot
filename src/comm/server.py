"""WebSocket/HTTP 通信服务:只做协议解析与转发,业务逻辑全部委托 core。

路由(架构 §5):
- WS  /ws            遥控主通道,每条合法指令处理后回发 state 兼作 ack
- GET /              H5 遥控页(web/index.html)
- GET /api/status    当前状态(与 state 消息同构)
- POST /api/drive|stop|patrol  调试端点,body 与 WS 消息同构

断连保护:WS 断开 → 停车并退出巡逻;控制循环以 50ms 节拍驱动
巡逻 tick 与看门狗 check(远小于 0.5s 超时,保证停车及时)。
"""
import asyncio
import json
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

from ..core.motion import mix
from ..core.patrol import Patrol
from ..core.safety import Watchdog
from ..platforms import create_drivetrain
from . import protocol

log = logging.getLogger(__name__)

WEB_INDEX = Path(__file__).resolve().parents[2] / "web" / "index.html"
CONTROL_LOOP_INTERVAL = 0.05  # 控制循环节拍,秒


def _platform_name(drivetrain):
    """从实现类的模块路径取平台名(src.platforms.<name>.drivetrain)。"""
    return type(drivetrain).__module__.split(".")[-2]


def create_app(drivetrain=None, patrol=None, watchdog=None):
    """组装应用。缺省自动探测平台;测试时注入 mock 底盘/假时钟组件。"""
    drivetrain = drivetrain or create_drivetrain()
    patrol = patrol or Patrol(drivetrain)
    watchdog = watchdog or Watchdog(drivetrain)

    def state():
        return {
            "type": "state",
            "patrol": patrol.active,
            "platform": _platform_name(drivetrain),
        }

    def handle_drive(msg):
        patrol.handle_manual_drive()  # 手动优先:退出巡逻
        left, right = mix(msg["vx"], msg["wz"])
        drivetrain.set_speeds(left, right)
        watchdog.feed()

    def handle_stop(_msg):
        patrol.stop()
        drivetrain.stop()
        watchdog.feed()

    def handle_patrol(msg):
        if msg["on"]:
            patrol.start(msg.get("speed"), msg.get("period"))
        else:
            patrol.stop()
        watchdog.feed()

    dispatch = {"drive": handle_drive, "stop": handle_stop, "patrol": handle_patrol}

    async def control_loop():
        while True:
            patrol.tick()
            if patrol.active:
                watchdog.feed()  # 巡逻期间电机在执行有意指令,看门狗不触发
            watchdog.check()
            await asyncio.sleep(CONTROL_LOOP_INTERVAL)

    @asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(control_loop())
        try:
            yield
        finally:
            task.cancel()
            drivetrain.close()  # 退出(含异常)释放 GPIO

    app = FastAPI(lifespan=lifespan)

    @app.websocket("/ws")
    async def ws_endpoint(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                msg = protocol.parse_message(await websocket.receive_text())
                if msg is not None:
                    dispatch[msg["type"]](msg)
                    await websocket.send_text(json.dumps(state()))
        except WebSocketDisconnect:
            pass
        finally:
            patrol.stop()  # 断连:退出巡逻并停车(FR-3)
            drivetrain.stop()
            watchdog.feed()

    @app.get("/")
    async def index():
        return FileResponse(WEB_INDEX)

    @app.get("/api/status")
    async def api_status():
        return state()

    async def handle_debug_post(request, expected_type):
        msg = protocol.parse_message(await request.body())
        if msg is None or msg["type"] != expected_type:
            raise HTTPException(status_code=400, detail="非法消息")
        dispatch[msg["type"]](msg)
        return state()

    @app.post("/api/drive")
    async def api_drive(request: Request):
        return await handle_debug_post(request, "drive")

    @app.post("/api/stop")
    async def api_stop(request: Request):
        return await handle_debug_post(request, "stop")

    @app.post("/api/patrol")
    async def api_patrol(request: Request):
        return await handle_debug_post(request, "patrol")

    return app
