"""comm/server 端到端测试:内存中跑完整服务,底盘用 mock 记账。"""
import pytest
from fastapi.testclient import TestClient

from src.comm.server import create_app
from src.platforms.mock.drivetrain import MockDrivetrain

ODOM_ZERO = {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0.0, "wz": 0.0}


@pytest.fixture
def drivetrain():
    return MockDrivetrain()


@pytest.fixture
def client(drivetrain):
    # with 块触发 lifespan:启动控制循环,退出时 close() 释放资源
    with TestClient(create_app(drivetrain)) as client:
        yield client


def test_index_serves_h5(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert "<html" in resp.text.lower()


def test_status_endpoint(client):
    resp = client.get("/api/status")
    assert resp.json() == {"type": "state", "patrol": False, "platform": "mock",
                           "odom": ODOM_ZERO}


def test_ws_drive_moves_motors_and_acks(client, drivetrain):
    with client.websocket_connect("/ws") as ws:
        ws.send_text('{"type": "drive", "vx": 0.5, "wz": 0}')
        ack = ws.receive_json()
        assert ack == {"type": "state", "patrol": False, "platform": "mock",
                       "odom": ODOM_ZERO}
    assert (0.5, 0.5) in drivetrain.calls


def test_ws_disconnect_parks(client, drivetrain):
    with client.websocket_connect("/ws") as ws:
        ws.send_text('{"type": "drive", "vx": 0.5, "wz": 0}')
        ws.receive_json()
    assert drivetrain.calls[-1] == (0.0, 0.0)  # 断连即停车(FR-3)


def test_ws_invalid_message_ignored_connection_kept(client, drivetrain):
    with client.websocket_connect("/ws") as ws:
        ws.send_text("not json")
        ws.send_text('{"type": "drive", "vx": 0.3, "wz": 0}')
        assert ws.receive_json()["type"] == "state"  # 连接未断,后续指令照常处理
    assert (0.3, 0.3) in drivetrain.calls


def test_ws_manual_drive_overrides_patrol(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_text('{"type": "patrol", "on": true, "speed": 0.5, "period": 4.0}')
        assert ws.receive_json()["patrol"] is True
        ws.send_text('{"type": "drive", "vx": 0.2, "wz": 0}')
        assert ws.receive_json()["patrol"] is False  # 手动优先


def test_ws_stop_exits_patrol(client, drivetrain):
    with client.websocket_connect("/ws") as ws:
        ws.send_text('{"type": "patrol", "on": true}')
        ws.receive_json()
        ws.send_text('{"type": "stop"}')
        assert ws.receive_json()["patrol"] is False
    assert drivetrain.calls[-1] == (0.0, 0.0)


def test_http_debug_endpoints_match_ws(client, drivetrain):
    resp = client.post("/api/drive", json={"type": "drive", "vx": 0.4, "wz": 0})
    assert resp.status_code == 200
    assert (0.4, 0.4) in drivetrain.calls
    resp = client.post("/api/patrol", json={"type": "patrol", "on": True})
    assert resp.json()["patrol"] is True
    resp = client.post("/api/stop", json={"type": "stop"})
    assert resp.json()["patrol"] is False
    assert drivetrain.calls[-1] == (0.0, 0.0)


def test_http_debug_rejects_bad_body(client):
    assert client.post("/api/drive", content="not json").status_code == 400
    assert client.post("/api/drive", json={"type": "stop"}).status_code == 400  # 类型不匹配
