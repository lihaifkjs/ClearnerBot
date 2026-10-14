"""2D 射线投射仿真雷达(M8):对 M7 地图(pgm/yaml)产假 /scan。

地图解析与射线投射为纯函数,Mac 离线单测覆盖;
ROS 节点部分 rclpy/tf2 延迟导入,无 ROS 环境 import 本模块不受影响。

角度约定:第 i 束光的世界方向 = laser 位姿 yaw + i·2π/beams,
与真机 LD06 的差异由同一份静态 TF(base_link→laser_link,yaw=-π/2)吸收,
仿真/真机链路保持一致。

用法(Pi 上):python -m src.sim.raycast maps/house.yaml
"""
import logging
import math
import sys
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)

SCAN_BEAMS = 360
SCAN_HZ = 10.0
SCAN_RANGE_MIN = 0.05
SCAN_RANGE_MAX = 8.0  # 与 M7 slam 配置对齐(LD06 标称 12m,远点噪声大先收 8m)


@dataclass
class SimMap:
    """占用栅格地图:grid[row][col] 为 True 表示墙。

    坐标换算:pgm 像素行自上而下,地图原点 origin 在左下角,
    故世界坐标 (x, y) 所在像素 row = height-1-(y-origin_y)/resolution。
    """

    grid: list  # list[list[bool]],row 0 = pgm 首行(地图最北)
    resolution: float  # 米/格
    origin_x: float
    origin_y: float

    @property
    def height(self):
        return len(self.grid)

    @property
    def width(self):
        return len(self.grid[0]) if self.grid else 0

    def is_occupied(self, x, y):
        """世界坐标 → 是否墙;出界视为非墙(射线可穿过地图边缘)。"""
        col = int((x - self.origin_x) / self.resolution)
        row = self.height - 1 - int((y - self.origin_y) / self.resolution)
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        return self.grid[row][col]


def _read_pgm(path):
    """解析 8 位二进制 PGM(P5/maxval=255)→ (width, height, pixels bytes)。

    头部为 4 个空白分隔的 token(P5、宽、高、maxval),允许 # 注释行;
    maxval 后恰有 1 个空白字符,随后是 width×height 字节像素。
    """
    data = Path(path).read_bytes()
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] != b"\n":
                i += 1
            continue
        start = i
        while i < len(data) and not data[i : i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5" or maxval != 255:
        raise ValueError(f"仅支持 8 位二进制 PGM(P5/maxval=255):{path}")
    pixels = data[i : i + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"PGM 像素数据长度不符:{path}")
    return width, height, pixels


def _parse_map_yaml(text):
    """解析 map_server 地图 yaml 的最小子集(key: value,origin 为流式列表)。"""
    meta = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, _, value = line.partition(":")
        value = value.strip()
        if value.startswith("["):
            value = [float(v) for v in value.strip("[]").split(",")]
        else:
            try:
                value = float(value)
            except ValueError:
                pass  # image/mode 等字符串字段原样保留
        meta[key.strip()] = value
    return meta


def load_map(yaml_path):
    """读地图 yaml + pgm → SimMap。occ=(255-pixel)/255 > occupied_thresh 判墙(negate=0)。"""
    yaml_path = Path(yaml_path)
    meta = _parse_map_yaml(yaml_path.read_text(encoding="utf-8"))
    width, height, pixels = _read_pgm(yaml_path.parent / meta["image"])
    thresh = float(meta.get("occupied_thresh", 0.65))
    negate = int(meta.get("negate", 0))
    grid = []
    for row in range(height):
        line = []
        for col in range(width):
            pixel = pixels[row * width + col]
            occ = pixel / 255 if negate else (255 - pixel) / 255
            line.append(occ > thresh)
        grid.append(line)
    origin = meta["origin"]
    return SimMap(
        grid=grid,
        resolution=float(meta["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )


def cast_ray(sim_map, x, y, angle, max_range=SCAN_RANGE_MAX):
    """从 (x, y) 沿 angle 步进采样,命中墙返回距离;到 max_range 未命中返回 max_range。

    步长取半格(resolution/2):小于一格保证不穿透单格厚度的墙,
    又免去 DDA 的轴向退化处理;仿真精度 ±resolution/2,对建图分辨率足够。
    """
    step = sim_map.resolution / 2
    dx, dy = math.cos(angle), math.sin(angle)
    dist = step
    while dist < max_range:
        if sim_map.is_occupied(x + dist * dx, y + dist * dy):
            return dist
        dist += step
    return max_range


def make_scan(sim_map, x, y, yaw, beams=SCAN_BEAMS, max_range=SCAN_RANGE_MAX):
    """生成一圈距离数组;第 i 束方向 = yaw + i·2π/beams。"""
    return [
        cast_ray(sim_map, x, y, yaw + i * 2 * math.pi / beams, max_range)
        for i in range(beams)
    ]


def _import_ros():
    """延迟导入 ROS 依赖;无 ROS 环境返回 None。独立成函数便于测试替换。"""
    try:
        import rclpy
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        return None
    return rclpy, Time, LaserScan, Buffer, TransformListener


class RaycastSim:
    """假激光节点:查 laser_link 位姿,对地图射线投射,10Hz 发 /scan。

    优先查 map→laser_link;AMCL 未启动时 map→odom 不存在,回退查
    odom→laser_link(仿真里程计无漂移,map≈odom,供 AMCL 冷启动收敛)。
    """

    def __init__(self, sim_map, beams=SCAN_BEAMS, max_range=SCAN_RANGE_MAX):
        self._map = sim_map
        self._beams = beams
        self._max_range = max_range
        self._rclpy = None
        self._Time = None
        self._LaserScan = None
        self._node = None
        self._tf_buffer = None
        self._pub = None

    def start(self):
        """启动节点;无 ROS 环境或已启动时返回 False。"""
        if self._node is not None:
            return False
        modules = _import_ros()
        if modules is None:
            log.warning("rclpy/sensor_msgs/tf2_ros 不可用(非 ROS 环境),仿真雷达未启动")
            return False
        self._rclpy, self._Time, self._LaserScan, Buffer, TransformListener = modules
        self._rclpy.init()
        self._node = self._rclpy.create_node("clearnerbot_raycast_sim")
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer, self._node)
        self._pub = self._node.create_publisher(self._LaserScan, "/scan", 10)
        self._node.create_timer(1.0 / SCAN_HZ, self._publish_once)
        log.info("仿真雷达已启动:/scan %d 束 @%.0fHz(地图 %dx%d)",
                 self._beams, SCAN_HZ, self._map.width, self._map.height)
        return True

    def _laser_pose(self):
        """laser_link 在 map(回退 odom)中的位姿 (x, y, yaw);TF 未就绪返回 None。"""
        for target in ("map", "odom"):
            try:
                tf = self._tf_buffer.lookup_transform(target, "laser_link", self._Time())
            except Exception:  # TF 缺失/外推失败等,换下一个目标系
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            # 四元数 → 偏航角(平面底盘只有 z 轴旋转,通用公式以防万一)
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return t.x, t.y, yaw
        return None

    def _publish_once(self):
        pose = self._laser_pose()
        if pose is None:
            return
        x, y, yaw = pose
        msg = self._LaserScan()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / self._beams
        msg.angle_max = msg.angle_increment * (self._beams - 1)
        msg.range_min = SCAN_RANGE_MIN
        msg.range_max = self._max_range
        msg.ranges = make_scan(self._map, x, y, yaw, self._beams, self._max_range)
        self._pub.publish(msg)

    def spin(self):
        self._rclpy.spin(self._node)

    def stop(self):
        """停节点;幂等。"""
        if self._node is None:
            return
        self._node.destroy_node()
        self._rclpy.shutdown()
        self._node = None
        self._rclpy = None
        self._tf_buffer = None
        self._pub = None


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    default_map = Path(__file__).resolve().parents[2] / "maps" / "house.yaml"
    map_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_map
    sim = RaycastSim(load_map(map_path))
    if not sim.start():
        raise SystemExit("rclpy 不可用,请在 ROS 环境(conda activate ros)下运行")
    try:
        sim.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()


if __name__ == "__main__":
    main()
