from __future__ import annotations

import math

import numpy as np
import rclpy
from amr_interfaces.msg import SafetyState
from geometry_msgs.msg import Point, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from amr_safety.safety_fields import (EMERGENCY_STOP, PROTECTIVE_STOP, STATE_NAMES, WARNING,
                                      FieldConfig, SafetyMonitor, limit_command, scan_to_points)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Safety field monitor; the only node allowed to command the wheels
class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_monitor")

        # Every FieldConfig value is exposed as a ROS parameter
        cfg = FieldConfig()
        for name, default in vars(cfg).items():
            setattr(cfg, name, self.declare_parameter(name, default).value)
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.5).value
        rate = self.declare_parameter("rate", 20.0).value

        self.monitor = SafetyMonitor(cfg)
        self.points = np.empty((0, 2))
        self.sensor_tf: tuple[float, float, float] | None = None
        self.cmd = Twist()
        self.cmd_stamp = None
        self.measured_v = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Inputs: lidar, requested velocity, measured velocity, e-stop
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_smoothed", self.on_cmd, 10)
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10)
        self.create_subscription(Bool, "safety/estop", self.on_estop, 10)

        # Outputs: wheel command, safety state, field visualisation, e-stop reset
        self.cmd_pub = self.create_publisher(TwistStamped, "diff_drive_controller/cmd_vel", 10)
        self.state_pub = self.create_publisher(SafetyState, "safety/state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "safety/fields", 10)
        self.create_service(Trigger, "safety/reset", self.on_reset)

        self.create_timer(1.0 / rate, self.tick)
        self.prev_state = None
        self.get_logger().info("Safety monitor running")

    def on_scan(self, msg: LaserScan):
        # The lidar mounting pose is static, so it is looked up once
        if self.sensor_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time(),
                                                    timeout=Duration(seconds=0.1))
            except TransformException as exc:
                self.get_logger().warn(f"waiting for TF {self.base_frame}->{msg.header.frame_id}: {exc}",
                                       throttle_duration_sec=2.0)
                return
            tr = t.transform
            self.sensor_tf = (tr.translation.x, tr.translation.y, _yaw(tr.rotation))
        sx, sy, syaw = self.sensor_tf
        self.points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, sx, sy, syaw)
        self.monitor.scan_received(self.now())

    def on_cmd(self, msg: Twist):
        self.cmd = msg
        self.cmd_stamp = self.now()

    def on_odom(self, msg: Odometry):
        self.measured_v = msg.twist.twist.linear.x

    def on_estop(self, msg: Bool):
        if msg.data and not self.monitor.estop:
            self.get_logger().error("E-STOP pressed")
            self.monitor.press_estop()

    def on_reset(self, _req, res):
        res.success = self.monitor.reset_estop()
        res.message = "e-stop reset" if res.success else "e-stop was not active"
        return res

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # Control loop: evaluate the fields, limit the command, publish state and markers
    def tick(self):
        now = self.now()
        cmd_v, cmd_w = self.cmd.linear.x, self.cmd.angular.z
        if self.cmd_stamp is None or now - self.cmd_stamp > self.cmd_timeout:
            cmd_v, cmd_w = 0.0, 0.0

        # Fields are sized for the larger of the requested and the measured speed
        v_field = max(cmd_v, self.measured_v, key=abs)
        ev = self.monitor.update(self.points, v_field, cmd_w, now)
        v, w = limit_command(cmd_v, cmd_w, ev)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.twist.linear.x, out.twist.angular.z = v, w
        self.cmd_pub.publish(out)

        st = SafetyState()
        st.header = out.header
        st.state = ev.state
        st.speed_scale = float(ev.speed_scale)
        st.closest_obstacle_distance = float(ev.closest_distance)
        st.protective_field_length = float(ev.fields.protective_length)
        st.warning_field_length = float(ev.fields.warning_length)
        st.reason = ev.reason
        self.state_pub.publish(st)
        self.publish_markers(ev)

        # rclpy fixes the severity per call site, so each severity needs its own logging line
        if ev.state != self.prev_state:
            message = f"safety state -> {STATE_NAMES[ev.state]} ({ev.reason})"
            if ev.state in (PROTECTIVE_STOP, EMERGENCY_STOP):
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)
            self.prev_state = ev.state

    # Warning field in yellow, protective field in red; highlighted when active
    def publish_markers(self, ev):
        cfg = self.monitor.cfg
        hw = cfg.footprint_half_width + cfg.lateral_margin
        arr = MarkerArray()
        for i, (length, rgba) in enumerate([
            (ev.fields.warning_length, (1.0, 0.8, 0.0, 0.25 if ev.state != WARNING else 0.6)),
            (ev.fields.protective_length, (1.0, 0.0, 0.0, 0.3 if ev.state < PROTECTIVE_STOP else 0.7)),
        ]):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "fields", i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            x0 = cfg.footprint_front if ev.fields.forward else -cfg.footprint_rear
            x1 = x0 + length if ev.fields.forward else x0 - length
            for x, y in [(x0, hw), (x1, hw), (x1, -hw), (x0, -hw), (x0, hw)]:
                m.points.append(Point(x=x, y=y, z=0.05))
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the robot with a stale command
        node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
