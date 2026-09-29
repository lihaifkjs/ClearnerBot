"""comm/protocol 表驱动测试:合法解析、越界截断、非法消息返回 None。"""
import pytest

from src.comm.protocol import parse_message


@pytest.mark.parametrize(
    "raw, expected",
    [
        ('{"type": "drive", "vx": 0.6, "wz": -0.2}', {"type": "drive", "vx": 0.6, "wz": -0.2}),
        ('{"type": "drive", "vx": 5, "wz": -5}', {"type": "drive", "vx": 1.0, "wz": -1.0}),  # 越界截断
        ('{"type": "stop"}', {"type": "stop"}),
        ('{"type": "patrol", "on": true, "speed": 0.5, "period": 4.0}',
         {"type": "patrol", "on": True, "speed": 0.5, "period": 4.0}),
        ('{"type": "patrol", "on": true}', {"type": "patrol", "on": True}),  # 参数缺省=沿用
        ('{"type": "patrol", "on": true, "speed": 99, "period": 0.01}',
         {"type": "patrol", "on": True, "speed": 1.0, "period": 1.0}),  # 越界截断
        ('{"type": "patrol", "on": false}', {"type": "patrol", "on": False}),
        ('{"type": "patrol", "on": false, "speed": "x"}', {"type": "patrol", "on": False}),  # 关闭时忽略参数
    ],
)
def test_valid_messages(raw, expected):
    assert parse_message(raw) == expected


@pytest.mark.parametrize(
    "raw",
    [
        "",  # 空
        "not json",  # 非法 JSON
        "[1, 2]",  # 非对象
        '{"vx": 0.5}',  # 缺 type
        '{"type": "fly"}',  # 未知类型
        '{"type": "drive", "vx": 0.5}',  # drive 缺 wz
        '{"type": "drive", "vx": "fast", "wz": 0}',  # 非数值
        '{"type": "drive", "vx": true, "wz": 0}',  # bool 不算数值
        '{"type": "patrol"}',  # patrol 缺 on
        '{"type": "patrol", "on": "yes"}',  # on 非 bool
        '{"type": "patrol", "on": true, "speed": "fast"}',  # 给了 speed 但不是数值
        "null",
    ],
)
def test_invalid_messages_ignored(raw):
    assert parse_message(raw) is None


def test_accepts_bytes():
    assert parse_message(b'{"type": "stop"}') == {"type": "stop"}
