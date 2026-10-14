"""航点巡逻 CLI(M8):读航点 JSON → Nav2 FollowWaypoints action 循环巡逻。

航点 JSON 是 M8/M9 共用契约(见《M8 导航巡逻.md》§5.4):
{"frame": "map", "loop": true, "waypoints": [{"x": 1.2, "y": 0.6, "theta": 0.0}]}

用法(Pi 上):python -m src.tools.nav_patrol waypoints.json
rclpy/action 依赖延迟导入,Mac 离线可测纯函数与巡逻流程。
"""
import json
import logging
import math
import sys
from pathlib import Path

from ..comm.ros_bridge import yaw_to_quaternion

log = logging.getLogger(__name__)

ACTION_NAME = "follow_waypoints"


def load_waypoints(path):
    """读取并校验航点 JSON → {"loop": bool, "waypoints": [(x, y, theta), ...]};
    非法输入抛 ValueError。"""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except OSError as e:
        raise ValueError(f"航点文件读不到:{e}") from e
    except ValueError as e:
        raise ValueError(f"航点文件不是合法 JSON:{e}") from e
    if not isinstance(data, dict):
        raise ValueError("航点文件顶层必须是 JSON 对象")
    if data.get("frame") != "map":
        raise ValueError('frame 必须为 "map"(航点定义在地图坐标系)')
    loop = data.get("loop")
    if not isinstance(loop, bool):
        raise ValueError("loop 必须为 true/false")
    raw = data.get("waypoints")
    if not isinstance(raw, list) or not raw:
        raise ValueError("waypoints 必须是非空列表")
    waypoints = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"第 {i + 1} 个航点必须是对象")
        point = []
        for key in ("x", "y", "theta"):
            value = item.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"第 {i + 1} 个航点缺少数值字段 {key}")
            if not math.isfinite(value):
                raise ValueError(f"第 {i + 1} 个航点 {key} 必须是有限数值")
            point.append(float(value))
        waypoints.append(tuple(point))
    return {"loop": loop, "waypoints": waypoints}


def waypoints_to_pose_data(waypoints):
    """[(x, y, theta)] → [{"x", "y", "qz", "qw"}];平面位姿只有 z 轴旋转。"""
    poses = []
    for x, y, theta in waypoints:
        _, _, qz, qw = yaw_to_quaternion(theta)
        poses.append({"x": x, "y": y, "qz": qz, "qw": qw})
    return poses


def run_patrol(send_waypoints, data, log_fn=print):
    """巡逻主循环(与 ROS 解耦,便于单测)。

    send_waypoints(poses) → 未到达航点下标列表(missed);
    loop=true 时循环发送,KeyboardInterrupt 由调用方(main)取消 goal 后退出。
    """
    poses = waypoints_to_pose_data(data["waypoints"])
    total = len(poses)
    round_no = 0
    while True:
        round_no += 1
        missed = send_waypoints(poses)
        if missed:
            log_fn(f"第 {round_no} 圈完成:{total - len(missed)}/{total} 到达,未到达 {missed}")
        else:
            log_fn(f"第 {round_no} 圈完成:{total}/{total} 全部到达")
        if not data["loop"]:
            return missed


def _import_ros():
    """延迟导入 ROS action 依赖;无 ROS 环境返回 None。独立成函数便于测试替换。"""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import FollowWaypoints
        from rclpy.action import ActionClient
    except ImportError:
        return None
    return rclpy, FollowWaypoints, PoseStamped, ActionClient


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    if len(sys.argv) < 2:
        raise SystemExit("用法:python -m src.tools.nav_patrol <waypoints.json>")
    try:
        data = load_waypoints(sys.argv[1])
    except ValueError as e:
        raise SystemExit(f"航点文件无效:{e}")
    modules = _import_ros()
    if modules is None:
        raise SystemExit("rclpy/nav2_msgs 不可用,请在 ROS 环境(conda activate ros)下运行")
    rclpy, FollowWaypoints, PoseStamped, ActionClient = modules

    rclpy.init()
    node = rclpy.create_node("clearnerbot_nav_patrol")
    client = ActionClient(node, FollowWaypoints, ACTION_NAME)
    goal_handle = None

    def send_waypoints(poses):
        nonlocal goal_handle
        while not client.wait_for_server(timeout_sec=1.0):
            log.info("等待 Nav2 %s action server ...", ACTION_NAME)
        goal = FollowWaypoints.Goal()
        for p in poses:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = p["x"]
            pose.pose.position.y = p["y"]
            pose.pose.orientation.z = p["qz"]
            pose.pose.orientation.w = p["qw"]
            goal.poses.append(pose)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("FollowWaypoints goal 被拒")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        return list(result_future.result().result.missed_waypoints)

    try:
        run_patrol(send_waypoints, data, log_fn=log.info)
    except KeyboardInterrupt:
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        log.info("已取消巡逻,退出(Nav2 停发 /cmd_vel 后看门狗 0.5s 停车)")
    except RuntimeError as e:
        raise SystemExit(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
