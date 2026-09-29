"""车端入口:组装 platforms → core → comm 三层,启动通信服务。

用法:python3 -m src.main
  树莓派:自动探测 GPIO 使用 raspberrypi 平台,手机浏览器访问 http://<树莓派IP>:8000/
  开发机:无 GPIO 自动回退 mock 仿真,指令打印到终端(先仿真后实物,FR-5)
树莓派上由 systemd 常驻拉起、异常自动重启(PRD FR-4)。
"""
import logging

import uvicorn

from src.comm.server import create_app


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    uvicorn.run(create_app(), host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
