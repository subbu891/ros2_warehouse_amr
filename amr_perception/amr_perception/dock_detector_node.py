import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from amr_perception.dock_detector import DockConfig, detect_dock, ranges_to_points


# Publishes the docked robot pose measured from the dock face in the lidar scan, for the Nav2 docking server
class DockDetectorNode(Node):
    def __init__(self):
        super().__init__("dock_detector")
        cfg = DockConfig()
        for name, default in vars(cfg).items():
            setattr(cfg, name, self.declare_parameter(name, default).value)
        self.cfg = cfg
        self.active = False
        self.found = None

        # Detection only runs during docking, when the robot faces the dock from the staging pose
        self.create_subscription(Bool, "safety/docking_mode", self.on_mode, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped, "detected_dock_pose", 10)

    def on_mode(self, msg: Bool):
        self.active = msg.data
        self.found = None

    def on_scan(self, msg: LaserScan):
        if not self.active:
            return
        det = detect_dock(ranges_to_points(msg.ranges, msg.angle_min, msg.angle_increment), self.cfg)
        if (det is not None) != self.found:
            self.found = det is not None
            if self.found:
                self.get_logger().info(f"dock detected, face width {det.width:.2f} m")
            else:
                self.get_logger().warn("dock not visible")
        if det is None:
            return

        # Pose in the lidar frame at the scan time; the docking server transforms it into its fixed frame
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = det.x, det.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(det.yaw / 2), math.cos(det.yaw / 2)
        self.pub.publish(pose)


def main():
    rclpy.init()
    node = DockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
