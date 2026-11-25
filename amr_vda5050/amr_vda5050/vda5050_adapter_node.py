from __future__ import annotations

import json
import math
import queue

import rclpy
from amr_interfaces.action import ExecuteMission
from amr_interfaces.msg import SafetyState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from amr_vda5050.mqtt_compat import make_client
from amr_vda5050.order_manager import OrderManager, topic


class Vda5050Adapter(Node):
    def __init__(self):
        super().__init__("vda5050_adapter")
        p = self.declare_parameter
        self.manufacturer = p("manufacturer", "TUM-Student").value
        self.serial = p("serial_number", "amr_001").value
        host, port = p("broker_host", "localhost").value, p("broker_port", 1883).value
        interface, major = p("interface_name", "uagv").value, p("major_version", "v2").value
        self.state_period = p("state_period", 1.0).value
        self.battery = p("initial_battery", 90.0).value
        self.drain_per_s = p("battery_drain_per_s", 0.02).value
        self.charge_per_s = p("battery_charge_per_s", 0.5).value

        self.om = OrderManager(self.manufacturer, self.serial, map_id=p("map_id", "warehouse").value)
        self.t = {k: topic(interface, major, self.manufacturer, self.serial, k)
                  for k in ("order", "instantActions", "state", "connection")}

        self.pose = None
        self.cov_xy = None
        self.vel = (0.0, 0.0)
        self.safety = SafetyState()
        self.charging = False
        # None, "pending" or a ClientGoalHandle
        self.goal_handle = None
        self.cancel_when_accepted = False
        self.inbox: queue.Queue = queue.Queue()

        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_pose, 10)
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10)
        self.create_subscription(SafetyState, "safety/state", self.on_safety, 10)
        self.mission = ActionClient(self, ExecuteMission, "execute_mission")

        self.mqtt = make_client(f"{self.manufacturer}-{self.serial}")
        self.mqtt.will_set(self.t["connection"], json.dumps(self.om.connection("CONNECTIONBROKEN")),
                           qos=1, retain=True)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = lambda _c, _u, m: self.inbox.put((m.topic, m.payload))
        self.mqtt.connect_async(host, port, keepalive=15)
        self.mqtt.loop_start()

        self.create_timer(0.1, self.process)
        self.create_timer(self.state_period, self.publish_state)
        self.create_timer(1.0, self.update_battery)
        self.get_logger().info(f"VDA 5050 adapter for {self.manufacturer}/{self.serial} -> {host}:{port}")

    # ROS inputs
    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.cov_xy = msg.pose.covariance[0] + msg.pose.covariance[7]

    def on_odom(self, msg):
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_safety(self, msg):
        self.safety = msg

    # MQTT
    # Accepts the callback arguments of paho-mqtt 1.x and 2.x
    def on_connect(self, client, _userdata, _flags, reason_code, *_props):
        self.get_logger().info(f"MQTT connected ({reason_code})")
        client.subscribe([(self.t["order"], 1), (self.t["instantActions"], 1)])
        client.publish(self.t["connection"], json.dumps(self.om.connection("ONLINE")), qos=1, retain=True)

    # Runs on the ROS thread: handle queued MQTT messages, then start the next task
    def process(self):
        changed = False
        while not self.inbox.empty():
            tp, payload = self.inbox.get()
            try:
                msg = json.loads(payload)
            except json.JSONDecodeError:
                self.get_logger().error(f"invalid JSON on {tp}")
                continue
            if tp == self.t["order"]:
                xy = self.pose[:2] if self.pose else None
                errs = self.om.receive_order(msg, xy)
                order_ref = f"order {msg.get('orderId')}/{msg.get('orderUpdateId')}"
                if errs:
                    self.get_logger().warn(f"{order_ref}: rejected {[e['errorDescription'] for e in errs]}")
                else:
                    self.get_logger().info(f"{order_ref}: accepted")
            elif tp == self.t["instantActions"]:
                if "cancel" in self.om.instant_actions(msg):
                    if hasattr(self.goal_handle, "cancel_goal_async"):
                        self.goal_handle.cancel_goal_async()
                    else:
                        self.cancel_when_accepted = True
                    self.get_logger().warn("order canceled by fleet manager")
            changed = True

        if self.om.executing is None and self.goal_handle is None:
            task = self.om.next_task(self.pose[:2] if self.pose else None)
            if task is not None:
                self.send_task(task)
                changed = True
        if changed:
            self.publish_state()

    def send_task(self, task):
        if not self.mission.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("execute_mission server not available yet", throttle_duration_sec=5.0)
            self.om.task_failed("mission executor not available")
            return
        goal = ExecuteMission.Goal(mission_id=f"{self.om.order_id}/{task.node_id}", steps=task.steps)
        self.get_logger().info(f"-> node {task.node_id}: {task.steps}")
        self.goal_handle = "pending"
        fut = self.mission.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.goal_handle = None
            self.om.task_failed("mission rejected by executor")
            return
        self.goal_handle = gh
        if self.cancel_when_accepted:
            self.cancel_when_accepted = False
            gh.cancel_goal_async()
        self.om.task_started()
        self.charging = False
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        res = fut.result().result
        self.goal_handle = None
        # The order was canceled while the mission was running
        if self.om.executing is None:
            return
        if res.success:
            if any(s.startswith("dock:") for s in self.om.executing.steps):
                self.charging = True
            self.om.task_succeeded()
        else:
            self.om.task_failed(res.message)
        self.publish_state()

    # State
    def update_battery(self):
        if self.charging:
            self.battery = min(100.0, self.battery + self.charge_per_s)
        elif abs(self.vel[0]) > 0.01:
            self.battery = max(0.0, self.battery - self.drain_per_s)

    def publish_state(self):
        estop = "MANUAL" if self.safety.state == SafetyState.EMERGENCY_STOP else "NONE"
        field_violation = self.safety.state == SafetyState.PROTECTIVE_STOP
        score = None if self.cov_xy is None else max(0.0, min(1.0, 1.0 - self.cov_xy / 0.5))
        state = self.om.build_state(self.pose, self.vel, abs(self.vel[0]) > 0.01 or abs(self.vel[1]) > 0.01,
                                    self.battery, self.charging, estop, field_violation, score)
        self.mqtt.publish(self.t["state"], json.dumps(state), qos=0)

    def shutdown(self):
        self.mqtt.publish(self.t["connection"], json.dumps(self.om.connection("OFFLINE")), qos=1, retain=True)
        self.mqtt.loop_stop()
        self.mqtt.disconnect()


def main():
    rclpy.init()
    node = Vda5050Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
