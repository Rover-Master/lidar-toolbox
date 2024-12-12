import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.publisher import Publisher
from rclpy.executors import ShutdownException, ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformListener,
    TFMessage,
    Buffer as TFBuffer,
)
from geometry_msgs.msg import PoseStamped, Quaternion
from functools import cached_property
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
    MetadataIo,
    BagMetadata,
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from math import atan2, asin, sin, cos, degrees
from pathlib import Path
from collections import deque
import time, os, json

CWD = Path(os.getcwd())
INITIAL_POSE_FILE = CWD / "INITIAL_POSE.conf"
MAP_COMPLETE_FLAG = CWD / "MAP_COMPLETE.flag"
TRJ_COMPLETE_FLAG = CWD / "TRJ_COMPLETE.flag"


class Attitude:
    def __init__(self, q: Quaternion):
        self.x = q.x
        self.y = q.y
        self.z = q.z
        self.w = q.w

    @cached_property
    def rx(self):
        return atan2(
            2.0 * (self.w * self.x + self.y * self.z),
            1.0 - 2.0 * (self.x * self.x + self.y * self.y),
        )

    @cached_property
    def ry(self):
        return asin(2.0 * (self.w * self.y - self.z * self.x))

    @cached_property
    def rz(self):
        return atan2(
            2.0 * (self.w * self.z + self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        )

    @staticmethod
    def to_quaternion(rx=0.0, ry=0.0, rz=0.0):
        cy = cos(rz * 0.5)
        sy = sin(rz * 0.5)
        cp = cos(ry * 0.5)
        sp = sin(ry * 0.5)
        cr = cos(rx * 0.5)
        sr = sin(rx * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


class TransformSub(TransformListener):
    def __init__(self, node: Node):
        super().__init__(buffer=TFBuffer(), node=node)
        self.node = node
        self.logger = node.get_logger()

    prev_info = None

    def callback(self, data: TFMessage):
        # Extract transform from "map" to "odom"
        transform: TransformStamped
        for transform in data.transforms:
            if (
                transform.child_frame_id == "odom"
                and transform.header.frame_id == "map"
            ):
                x, y, r = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    Attitude(transform.transform.rotation).rz,
                )
                info = f"T(map, odom): x {x:.2f}, y {y:.2f}, r {degrees(r):.2f} deg | {r:.2f} rad"
                if self.prev_info != info:
                    self.prev_info = info
                    self.logger.info(info)


class ScanPlayer(Node):

    class Param:
        @staticmethod
        def param(node: Node, name, default, type=lambda x: x):
            return type(node.declare_parameter(name, default).value)

        def __init__(self, node: Node):
            self.playback_speed = +self.param(node, "playback_speed", 1.0, float)
            self.heading_offset = -self.param(node, "heading_offset", 0.0, float)
            self.odom_direction = -self.param(node, "odom_direction", 0.0, float)
            self.bag_path = Path(self.param(node, "bag_path", "", str)).absolute()
            self.save_map = self.param(node, "save_map", False, bool)
            self.save_trj = self.param(node, "save_trj", False, bool)

    class Pub:
        def __init__(self, node: Node):
            self.tf = TransformBroadcaster(node)
            self.tf_static = StaticTransformBroadcaster(node)
            self.pose = node.create_publisher(PoseStamped, "/odom_pose", 10)
            self.scan = node.create_publisher(LaserScan, "/scan", 10)

    class Sub:
        def __init__(self, node: Node):
            self.tf = TransformSub(node)

    class Srv:
        def __init__(self, node: Node):
            self.save_map = node.create_client(SaveMap, "/slam_toolbox/save_map")
            self.serialize = node.create_client(
                SerializePoseGraph, "/slam_toolbox/serialize_map"
            )

    def __init__(self):
        super().__init__("scan_player")

        self.param = self.Param(self)
        self.pub = self.Pub(self)
        self.sub = self.Sub(self)
        self.srv = self.Srv(self)

        self.get_logger().info("Waiting for slam_toolbox ...")
        self.srv.save_map.wait_for_service(None)
        time.sleep(1.0)

        laser_tf = TransformStamped()
        laser_tf.header.frame_id = "base"
        laser_tf.child_frame_id = "laser"
        laser_tf.transform.rotation.w = 1.0
        self.pub.tf_static.sendTransform(laser_tf)

    task_queue = deque[tuple[float, callable]](maxlen=10)

    def rotate_task(self, ddl: float):
        while len(self.task_queue) > 0:
            t, _ = self.task_queue[0]
            if t > ddl:
                break
            _, task = self.task_queue.popleft()
            task()
            self.update_clock()

    def schedule_task(self, ddl: float, task: callable):
        while len(self.task_queue) >= self.task_queue.maxlen:
            # No timeout is specified because we need at least one task to be executed
            rclpy.spin_once(self)
        self.task_queue.append((ddl, task))

    clock: Publisher | None = None
    timer: Timer = None
    T0: float = None  # initial time (real time)
    t0: float = None  # initial time (sim time)
    duration: float = None  # duration of the playback

    def start_clock(
        self, t0: float, duration: float, period_sec: float = 1e-3, qos: int = 10
    ):
        self.T0 = time.time()
        self.t0 = t0
        self.duration = duration
        self.clock = self.create_publisher(Clock, "/clock", qos)
        self.timer = self.create_timer(period_sec, self.update_clock)
        self.update_clock()
        self.timer = self.create_timer(1.0, self.report_progress)
        # Publish initial transform
        s, ns = divmod(t0, 1.0)
        transform = TransformStamped()
        stamp = transform.header.stamp
        stamp.sec = int(s)
        stamp.nanosec = int(ns * 1e9)
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base"
        transform.transform.rotation = Attitude.to_quaternion(rz=0.0)
        self.pub.tf.sendTransform(transform)

    def report_progress(self):
        dt1 = self.dt
        dt2 = self.duration - dt1
        if self.duration > 0:
            p1, p2 = divmod(100.0 * dt1 / self.duration, 1.0)
            p1, p2 = int(p1), int(p2 * 100)
            prog = f"{p1:02d}.{p2:02d}%"
        else:
            prog = "--.--%"

        def format(t: float):
            if t < 0:
                return "--:--.---"
            t, ms = divmod(t, 1.0)
            ms = int(ms * 1e3)
            m, s = divmod(int(t), 60)
            return f"{m:02d}:{s:02d}.{ms:03d}"

        self.get_logger().info(
            f"{format(dt1)} Played | {format(dt2)} Remaining | {prog} Complete"
        )

    @property
    def dT(self):
        return time.time() - self.T0

    @property
    def dt(self):
        return self.dT * self.param.playback_speed

    @property
    def t1(self):
        return self.t0 + self.dt

    def update_clock(self):
        self.rotate_task(self.t1)
        s, ns = divmod(self.t1, 1.0)
        clk = Clock()
        clk.clock.sec = int(s)
        clk.clock.nanosec = int(ns * 1e9)
        if self.clock is not None:
            self.clock.publish(clk)
        else:
            raise RuntimeError("Clock publisher is not initialized")

    def save_map(self):
        result = True
        # Save map
        if self.srv.save_map.wait_for_service(timeout_sec=0):
            request = SaveMap.Request()
            request.name = String(data="map")
            future = self.srv.save_map.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            res: SaveMap.Response = future.result()
            if res and res.result == SaveMap.Response.RESULT_SUCCESS:
                self.get_logger().info("Map saved successfully")
            else:
                self.get_logger().error("Failed to save map")
                result = False
        else:
            self.get_logger().error("SaveMap service is not available")
            result = False
        # Serialize pose graph
        if self.srv.serialize.wait_for_service(timeout_sec=0):
            request = SerializePoseGraph.Request()
            request.filename = "map"
            future = self.srv.serialize.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            res: SerializePoseGraph.Response = future.result()
            if res and res.result == SerializePoseGraph.Response.RESULT_SUCCESS:
                self.get_logger().info("Map serialized successfully")
            else:
                self.get_logger().error("Failed to serialize map")
                result = False
        else:
            self.get_logger().error("SerializePoseGraph service is not available")
            result = False
        return result

    flag_initial_pose = False

    def odometry(self, msg: Odometry):
        # Extract orientation
        rz = Attitude(msg.pose.pose.orientation).rz
        r = rz + self.param.heading_offset
        rot = Attitude.to_quaternion(rz=r)
        if self.flag_initial_pose:
            INITIAL_POSE_FILE.write_text(json.dumps([0.0, 0.0, rz]))
            self.flag_initial_pose = False
        # Extract position
        dr = self.param.heading_offset + self.param.odom_direction
        pos = msg.pose.pose.position
        x, y = pos.x, pos.y
        # Scaling factor
        K = 0.6
        # Apply rotation
        pos.x = K * (cos(dr) * x - sin(dr) * y)
        pos.y = K * (sin(dr) * x + cos(dr) * y)
        pos.z = 0.0
        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "odom"  # Parent frame
        transform.child_frame_id = "base"  # Child frame
        # Set translation
        transform.transform.translation.x = pos.x
        transform.transform.translation.y = pos.y
        transform.transform.translation.z = pos.z
        # Set rotation
        transform.transform.rotation = rot
        # Publish the pose
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose.position = pos
        pose.pose.orientation = rot

        # Compose the callback function
        def callback():
            self.pub.tf.sendTransform(transform)
            self.pub.pose.publish(pose)

        return callback

    def laser_scan(self, msg: LaserScan):
        # Perform obstacle filtering and then transform the scan data
        msg.header.frame_id = "laser"
        # Publish the transformed scan data to the /scan_transformed topic
        return lambda: self.pub.scan.publish(msg)


def suffix(name: str, default: str = None):
    segments = name.split(".")
    return segments[-1] if len(segments) > 1 else default


def parse_time(t: object):

    from builtin_interfaces.msg import Time
    from rclpy.time import Time as RclpyTime
    from rclpy.duration import Duration

    if isinstance(t, Time):
        return t.sec + t.nanosec * 1e-9
    elif isinstance(t, (RclpyTime, Duration)):
        return t.nanoseconds * 1e-9
    elif isinstance(t, float):
        return t
    else:
        raise ValueError(f"Unknown time format: ({type(t)}) {t}")


def main():
    rclpy.init()
    node = ScanPlayer()
    try:
        if not node.param.bag_path.exists() or not node.param.bag_path.is_file():
            node.get_logger().error(f"Bag file not found: {node.param.bag_path}")
            return
        reader = SequentialReader()
        node.get_logger().info(f"Opening bag file: {node.param.bag_path}")
        storage_options = StorageOptions(
            uri=str(node.param.bag_path),
            storage_id=suffix(node.param.bag_path.name, "mcap"),
        )
        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader.open(storage_options, converter_options)
        # Prepare bag metadata
        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        # Get start/end time
        try:
            bag_dir = str(node.param.bag_path.parent)
            metadata: BagMetadata = MetadataIo().read_metadata(bag_dir)
            # Convert timestamps to seconds for easier interpretation
            t0 = parse_time(metadata.starting_time)
            dt = parse_time(metadata.duration)
            node.start_clock(t0, dt)
        except Exception as e:
            node.get_logger().error(f"Failed to read metadata: {e}")
        # Config node before running
        if node.param.save_map:
            node.flag_initial_pose = True
        # Dump all messages
        while reader.has_next():
            topic, data, time_ns = reader.read_next()
            t = time_ns * 1e-9
            if node.clock is None:
                node.start_clock(t, 0.0)
            # Get message type and deserialize the message
            msg_type = topic_types[topic]
            msg = deserialize_message(data, get_message(msg_type))
            # Process the message, if necessary
            match msg_type:
                case "nav_msgs/msg/Odometry":
                    node.schedule_task(t, node.odometry(msg))
                case "sensor_msgs/msg/LaserScan":
                    node.schedule_task(t, node.laser_scan(msg))
            rclpy.spin_once(node, timeout_sec=1e-3)
        # Bag has been drained, wait 1 second and call save_map service if needed
        if node.param.save_map:
            # It might take a while for slam_toolbox to process the remaining data
            time.sleep(2.0)
            MAP_COMPLETE_FLAG.unlink(missing_ok=True)
            ret = node.save_map()
            if ret:
                MAP_COMPLETE_FLAG.touch()
        if node.param.save_trj:
            TRJ_COMPLETE_FLAG.unlink(missing_ok=True)
            ret = False
            if ret:
                TRJ_COMPLETE_FLAG.touch()
    except (KeyboardInterrupt, ShutdownException, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
