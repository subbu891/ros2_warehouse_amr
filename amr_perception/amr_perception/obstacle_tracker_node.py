from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from amr_perception.obstacle_tracker import (MultiTracker, StaticMap, TrackerConfig, cluster_points,
                                             predict_points, transform_points)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# XYZ float32 point cloud at a fixed height so the costmap obstacle layer accepts it
def make_cloud(points_xy: np.ndarray, header: Header, z: float = 0.5) -> PointCloud2:
    pts = np.zeros((len(points_xy), 3), dtype=np.float32)
    if len(points_xy):
        pts[:, :2] = points_xy
        pts[:, 2] = z
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate("xyz")]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


# Detects, tracks and predicts moving obstacles from the lidar and the known map
class ObstacleTrackerNode(Node):
    def __init__(self):
        super().__init__("obstacle_tracker")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.static_dilation = self.declare_parameter("static_dilation", 0.20).value
        self.cluster_gap = self.declare_parameter("cluster_gap", 0.3).value
        self.min_points = self.declare_parameter("min_cluster_points", 3).value
        self.max_range = self.declare_parameter("max_range", 8.0).value

        # Every TrackerConfig value is exposed as a ROS parameter
        cfg = TrackerConfig()
        for name, default in vars(cfg).items():
            setattr(cfg, name, self.declare_parameter(name, default).value)
        self.tracker = MultiTracker(cfg)
        self.static_map: StaticMap | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The map is latched by the map server, so subscribe with transient local durability
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_map, map_qos)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

        self.cloud_pub = self.create_publisher(PointCloud2, "perception/predicted_obstacles", 10)
        self.tracks_pub = self.create_publisher(PoseArray, "perception/tracks", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "perception/obstacle_markers", 10)

    def on_map(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.static_map = StaticMap(data, msg.info.resolution,
                                    (msg.info.origin.position.x, msg.info.origin.position.y),
                                    dilation_m=self.static_dilation)
        self.get_logger().info(f"static map received ({msg.info.width}x{msg.info.height})")

    # Scan pipeline: map frame, static filter, clustering, tracking, prediction
    def on_scan(self, msg: LaserScan):
        if self.static_map is None:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id,
                                                Time.from_msg(msg.header.stamp),
                                                timeout=Duration(seconds=0.1))
        except TransformException:
            return
        tr = t.transform
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(r.size)
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= min(msg.range_max, self.max_range))
        local = np.column_stack((r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])))
        world = transform_points(local, tr.translation.x, tr.translation.y, _yaw(tr.rotation))

        dynamic = world[~self.static_map.is_static(world)]
        clusters = cluster_points(dynamic, self.cluster_gap, self.min_points)
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.tracker.update(clusters, now)

        header = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        moving = self.tracker.moving()
        cfg = self.tracker.cfg
        self.cloud_pub.publish(make_cloud(predict_points(moving, cfg.horizon, cfg.horizon_step), header))
        self.publish_tracks(header)

    # Tracks as poses (heading = direction of motion) and as RViz cylinders: red moving, blue static
    def publish_tracks(self, header: Header):
        pa = PoseArray(header=header)
        markers = MarkerArray()
        markers.markers.append(Marker(header=header, action=Marker.DELETEALL))
        for t in self.tracker.confirmed():
            p = Pose()
            p.position.x, p.position.y = float(t.position[0]), float(t.position[1])
            yaw = math.atan2(t.velocity[1], t.velocity[0])
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            pa.poses.append(p)

            m = Marker(header=header, ns="tracks", id=t.track_id, type=Marker.CYLINDER, action=Marker.ADD)
            m.pose = p
            m.pose.position.z = 0.5
            m.scale.x = m.scale.y = 2 * t.radius
            m.scale.z = 1.0
            moving = t.speed >= self.tracker.cfg.moving_speed
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.2, 0.2, 0.6) if moving else (0.2, 0.6, 1.0, 0.4)
            m.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(m)
        self.tracks_pub.publish(pa)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = ObstacleTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
