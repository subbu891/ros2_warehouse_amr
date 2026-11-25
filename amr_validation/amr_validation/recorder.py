from __future__ import annotations

import csv
import math
import threading
from pathlib import Path

import numpy as np
from amr_interfaces.msg import SafetyState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# Simulation time of the measurement itself, not of its arrival
def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# Records ground truth, AMCL estimate and safety state for KPI evaluation
class Recorder:
    def __init__(self, node: Node):
        self.node = node
        self.lock = threading.Lock()
        self.active = False
        self.last_gt = None
        self.clear()
        node.create_subscription(Odometry, "ground_truth/odom", self._gt, 50)
        node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._amcl, 10)
        node.create_subscription(SafetyState, "safety/state", self._safety, 10)

    def clear(self):
        self.gt, self.est, self.safety = [], [], []

    def start(self):
        with self.lock:
            self.clear()
            self.active = True

    def stop(self):
        with self.lock:
            self.active = False

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    # The latest ground truth is always kept so the runner can check the robot pose between scenarios
    def _gt(self, msg):
        p = msg.pose.pose
        sample = (_stamp(msg), p.position.x, p.position.y, _yaw(p.orientation))
        with self.lock:
            self.last_gt = sample
            if self.active:
                self.gt.append(sample)

    def _amcl(self, msg):
        if self.active:
            p = msg.pose.pose
            with self.lock:
                self.est.append((_stamp(msg), p.position.x, p.position.y, _yaw(p.orientation)))

    def _safety(self, msg):
        if self.active:
            with self.lock:
                self.safety.append((_stamp(msg), msg.state, msg.closest_obstacle_distance, msg.speed_scale))

    # Results
    def arrays(self):
        with self.lock:
            gt = np.array(self.gt, dtype=float).reshape(-1, 4)
            est = np.array(self.est, dtype=float).reshape(-1, 4)
            saf = np.array(self.safety, dtype=float).reshape(-1, 4)
        return gt, est, saf

    def latest_gt(self):
        with self.lock:
            return self.last_gt

    def save(self, folder: Path, name: str):
        folder.mkdir(parents=True, exist_ok=True)
        gt, est, saf = self.arrays()
        for suffix, data, header in [("gt", gt, "t,x,y,yaw"), ("amcl", est, "t,x,y,yaw"),
                                     ("safety", saf, "t,state,closest,scale")]:
            with open(folder / f"{name}_{suffix}.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header.split(","))
                w.writerows(data.tolist())
