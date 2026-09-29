"""通信协议 v1:消息解析与字段校验(详见《软件架构设计.md》第 5 节)。

规则:参数越界截断(clamp),不信任客户端输入;
非法/无法解析的消息返回 None,由调用方忽略,不断连、不抛异常。
"""
import json

from ..core.patrol import MAX_PERIOD, MAX_SPEED, MIN_PERIOD, MIN_SPEED

DRIVE_MIN, DRIVE_MAX = -1.0, 1.0


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _number(data, key):
    """取数值字段;缺失或非数值(含 bool)返回 None。"""
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _parse_drive(data):
    vx = _number(data, "vx")
    wz = _number(data, "wz")
    if vx is None or wz is None:
        return None
    return {
        "type": "drive",
        "vx": _clamp(vx, DRIVE_MIN, DRIVE_MAX),
        "wz": _clamp(wz, DRIVE_MIN, DRIVE_MAX),
    }


def _parse_patrol(data):
    on = data.get("on")
    if not isinstance(on, bool):
        return None
    msg = {"type": "patrol", "on": on}
    if on:
        # speed/period 缺省表示沿用上次设置;给了就必须是合法数值,否则整条忽略
        for key, lo, hi in (("speed", MIN_SPEED, MAX_SPEED), ("period", MIN_PERIOD, MAX_PERIOD)):
            if key in data:
                value = _number(data, key)
                if value is None:
                    return None
                msg[key] = _clamp(value, lo, hi)
    return msg


def parse_message(raw):
    """解析一条客户端消息(str 或 bytes),返回 {"type": ...} 字典;非法返回 None。"""
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    msg_type = data.get("type")
    if msg_type == "drive":
        return _parse_drive(data)
    if msg_type == "stop":
        return {"type": "stop"}
    if msg_type == "patrol":
        return _parse_patrol(data)
    return None
