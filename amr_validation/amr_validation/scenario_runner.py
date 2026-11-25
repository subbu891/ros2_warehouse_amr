from __future__ import annotations

import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import yaml
from amr_interfaces.action import ExecuteMission
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, Trigger

from amr_validation import metrics as m
from amr_validation import scenario_checks as sc
from amr_validation.recorder import Recorder


# Call a Gazebo world service through the gz command-line tool
def gz_service(world: str, service: str, reqtype: str, req: str, timeout_ms: int = 3000) -> bool:
    cmd = ["gz", "service", "-s", f"/world/{world}/{service}", "--reqtype", reqtype,
           "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 2)
        return "true" in out.stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


# SDF of a static red box used as a test obstacle
def box_sdf(name: str, size) -> str:
    sx, sy, sz = size
    return (f"<sdf version='1.9'><model name='{name}'><static>true</static><link name='link'>"
            f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
            f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
            f"<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>"
            f"</visual></link></model></sdf>")


# Runs each scenario against the live simulation and evaluates its KPI checks
class ScenarioRunner(Node):
    def __init__(self, world: str, robot: str):
        # Simulation time keeps event timestamps consistent with the recorded message stamps
        super().__init__("scenario_runner", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.world, self.robot = world, robot
        self.rec = Recorder(self)
        self.mission = ActionClient(self, ExecuteMission, "execute_mission")
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.estop_pub = self.create_publisher(Bool, "safety/estop", 10)
        self.reset_cli = self.create_client(Trigger, "safety/reset")
        self.forklift_cli = self.create_client(SetBool, "forklift/enable")
        self.nomotion_cli = self.create_client(Empty, "request_nomotion_update")

        # Forklift motion is tracked to prove that the disturbance really happened
        self.forklift_y_range = None
        self.create_subscription(PoseStamped, "forklift/pose", self.on_forklift, 10)
        self.clear_clis = [self.create_client(ClearEntireCostmap, f"{c}/clear_entirely_{c}")
                           for c in ("global_costmap", "local_costmap")]
        self.spawned: list[str] = []

    def on_forklift(self, msg):
        if self.forklift_y_range is not None:
            y = msg.pose.position.y
            self.forklift_y_range = (min(self.forklift_y_range[0], y), max(self.forklift_y_range[1], y))

    # Helpers
    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"service {client.srv_name} not available")
            return None
        fut = client.call_async(request)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def teleport(self, x, y, yaw) -> bool:
        req = (f'name: "{self.robot}", position: {{x: {x}, y: {y}, z: 0.02}}, '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        return gz_service(self.world, "set_pose", "gz.msgs.Pose", req)

    # True when the ground-truth pose is within tol metres of (x, y)
    def at_position(self, x, y, tol=0.3) -> bool:
        g = self.rec.latest_gt()
        return g is not None and math.hypot(g[1] - x, g[2] - y) <= tol

    # Drive to a pose with the mission executor; used when teleporting is not possible
    def drive_to(self, x, y, yaw, timeout=240.0) -> bool:
        goal = ExecuteMission.Goal(mission_id="reset", steps=[f"goto_pose:{x},{y},{yaw}"])
        send = self.mission.send_goal_async(goal)
        end = time.time() + 10.0
        while not send.done() and time.time() < end:
            time.sleep(0.05)
        if not send.done() or not send.result().accepted:
            return False
        result = send.result().get_result_async()
        end = time.time() + timeout
        while not result.done() and time.time() < end:
            time.sleep(0.1)
        return result.done() and result.result().result.success

    # Place the robot at the scenario start: teleport and verify, otherwise drive there
    def reset_robot(self, x, y, yaw, error) -> bool:
        if self.teleport(x, y, yaw):
            time.sleep(1.5)
        if self.at_position(x, y):
            self.set_initial_pose(x + error[0], y + error[1], yaw + error[2])
        else:
            self.get_logger().warn("teleport unavailable, driving to the scenario start instead")
            if not self.drive_to(x, y, yaw):
                self.get_logger().error("could not reach the scenario start")
                return False
            if any(error):
                self.set_initial_pose(x + error[0], y + error[1], yaw + error[2])
        time.sleep(1.0)
        if not any(error):
            self.settle_localization()
        self.clear_costmaps()
        return True

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # Small initial uncertainty (sigma 0.1 m, 0.1 rad) because the true pose is known
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.2)

    # AMCL only updates while driving; forced updates let the particles converge before the scenario starts
    def settle_localization(self, updates=15):
        for _ in range(updates):
            self.call(self.nomotion_cli, Empty.Request(), timeout=1.0)
            time.sleep(0.2)

    def spawn_box(self, x, y, size):
        name = f"test_box_{len(self.spawned)}_{int(time.time())}"
        req = f'sdf: "{box_sdf(name, size)}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {size[2] / 2}}}}}'
        if gz_service(self.world, "create", "gz.msgs.EntityFactory", req):
            self.spawned.append(name)
            self.get_logger().info(f"spawned {name} at ({x:.2f}, {y:.2f})")

    def remove_spawned(self):
        for name in self.spawned:
            gz_service(self.world, "remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
        self.spawned.clear()

    def clear_costmaps(self):
        for cli in self.clear_clis:
            self.call(cli, ClearEntireCostmap.Request(), timeout=2.0)

    # One scenario
    def run(self, sc_def: dict, defaults: dict, layout: dict, out_dir: Path, tag: str) -> dict:
        self.get_logger().info(f"===== {tag}: {sc_def.get('description', '')}")
        timeout = sc_def.get("timeout", defaults.get("timeout", 240.0))

        # Fresh processes need a few seconds to discover services, so wait long enough before giving up
        needs_forklift = bool(sc_def.get("forklift", False))
        res = self.call(self.forklift_cli, SetBool.Request(data=needs_forklift), timeout=10.0)
        if needs_forklift and (res is None or not res.success):
            self.get_logger().error("forklift could not be enabled, scenario is invalid")
            return {"name": tag, "description": sc_def.get("description", ""), "passed": False,
                    "duration": 0.0, "message": "forklift could not be enabled",
                    "checks": [m.Check("forklift_enabled", 0.0, 1.0, ">=")]}
        self.call(self.reset_cli, Trigger.Request(), timeout=5.0)
        self.remove_spawned()
        if not self.mission.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("execute_mission action server not available - is the stack running?")
        x, y, yaw = sc_def["start"]
        self.reset_robot(x, y, yaw, sc_def.get("initial_pose_error", [0.0, 0.0, 0.0]))

        events = [dict(e, done=False) for e in sc_def.get("events", [])]
        run_events: dict = {}
        step_end: dict = {}
        current_step = {"i": -1}

        def on_feedback(fb):
            i = fb.feedback.step_index
            if i != current_step["i"]:
                if current_step["i"] >= 0 and (g := self.rec.latest_gt()):
                    step_end[current_step["i"]] = g[1:4]
                current_step["i"] = i

        self.rec.start()
        self.forklift_y_range = (math.inf, -math.inf) if needs_forklift else None
        t0 = time.time()
        goal = ExecuteMission.Goal(mission_id=tag, steps=sc_def["steps"])
        send = self.mission.send_goal_async(goal, feedback_callback=on_feedback)
        while not send.done():
            time.sleep(0.05)
        gh = send.result()
        result_future = gh.get_result_async() if gh.accepted else None

        success, message = False, "goal rejected" if not gh.accepted else ""
        while result_future is not None:
            elapsed = time.time() - t0
            self.process_events(events, elapsed, run_events)
            if result_future.done():
                res = result_future.result().result
                success, message = res.success, res.message
                break
            if elapsed > timeout:
                gh.cancel_goal_async()
                message = "timeout"
                break
            time.sleep(0.05)
        if (g := self.rec.latest_gt()) is not None and current_step["i"] >= 0:
            step_end.setdefault(current_step["i"], g[1:4])
        time.sleep(0.5)
        self.rec.stop()
        self.rec.save(out_dir, tag)
        if any(e["type"] == "estop" for e in events):
            self.estop_pub.publish(Bool(data=False))
            self.call(self.reset_cli, Trigger.Request(), timeout=1.0)

        gt, est, saf = self.rec.arrays()
        data = sc.RunData(gt, est, saf, success, message, time.time() - t0, run_events, step_end)
        checks = sc.evaluate(sc_def, defaults, data, layout)
        if needs_forklift:
            lo, hi = self.forklift_y_range
            checks.append(m.Check("forklift_travel_m", max(hi - lo, 0.0), 2.0, ">="))
        self.forklift_y_range = None
        passed = all(c.passed for c in checks)
        for c in checks:
            if c.passed:
                self.get_logger().info(str(c))
            else:
                self.get_logger().error(str(c))
        self.get_logger().info(f"===== {tag}: {'PASS' if passed else 'FAIL'} ({message})")
        return {"name": tag, "description": sc_def.get("description", ""), "passed": passed,
                "duration": data.duration, "message": message, "checks": checks}

    # Trigger timed or position-based disturbances: e-stop, static and sudden obstacles
    def process_events(self, events, elapsed, run_events):
        gt = self.rec.latest_gt()
        for e in events:
            if e["done"] and e["type"] != "estop":
                continue
            if e["type"] == "estop":
                if not e["done"] and elapsed >= e["at"]:
                    self.estop_pub.publish(Bool(data=True))
                    run_events["estop_t"] = self.rec.now()
                    e["done"], e["released"] = True, False
                    self.get_logger().warn("event: E-STOP")
                elif e["done"] and not e["released"] and elapsed >= e["at"] + e.get("hold", 3.0):
                    self.estop_pub.publish(Bool(data=False))
                    self.call(self.reset_cli, Trigger.Request(), timeout=1.0)
                    e["released"] = True
                    self.get_logger().info("event: e-stop reset")
            elif e["type"] == "spawn_box" and elapsed >= e.get("at", 0.0):
                self.spawn_box(*e["pose"], e["size"])
                e["done"] = True
            elif e["type"] == "spawn_box_ahead" and gt is not None and gt[1] > e["when_x_gt"]:
                _, x, y, yaw = gt
                # Distance is measured from the robot front edge
                d = e["distance"] + 0.3 + e["size"][0] / 2
                self.spawn_box(x + d * math.cos(yaw), y + d * math.sin(yaw), e["size"])
                run_events["box_t"] = self.rec.now()
                e["done"] = True


# Run the selected scenarios, write CSV logs, summary.json and report.md; exit code 0 only if all pass
def main(argv=None):
    share = Path(get_package_share_directory("amr_validation"))
    ap = argparse.ArgumentParser(description="Automated scenario test suite for the warehouse AMR simulation")
    ap.add_argument("--scenarios", default=str(share / "config" / "scenarios.yaml"))
    ap.add_argument("--layout", default=str(Path(get_package_share_directory("amr_gazebo"))
                                            / "config" / "warehouse_layout.yaml"))
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--output", default=f"test_results/{time.strftime('%Y%m%d_%H%M%S')}")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--robot", default="amr")
    args, _ = ap.parse_known_args(argv)

    spec = yaml.safe_load(Path(args.scenarios).read_text())
    layout = yaml.safe_load(Path(args.layout).read_text())
    selected = [s for s in spec["scenarios"] if not args.only or s["name"] in args.only]
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ScenarioRunner(args.world, args.robot)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    results = []
    try:
        for s in selected:
            for k in range(args.repeat):
                tag = s["name"] if args.repeat == 1 else f"{s['name']}_{k + 1:02d}"
                results.append(node.run(s, spec.get("defaults", {}), layout, out_dir, tag))
    except KeyboardInterrupt:
        pass
    finally:
        node.remove_spawned()
        node.call(node.forklift_cli, SetBool.Request(data=False), timeout=1.0)

    (out_dir / "report.md").write_text(sc.report_markdown(results))
    (out_dir / "summary.json").write_text(json.dumps(
        [{**r, "checks": [{"name": c.name, "value": c.value, "limit": c.limit, "passed": c.passed}
                          for c in r["checks"]]} for r in results], indent=2))
    n_pass = sum(r["passed"] for r in results)
    print(f"\n{n_pass}/{len(results)} scenarios passed. Report: {out_dir / 'report.md'}")
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if results and n_pass == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
