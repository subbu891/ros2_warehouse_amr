from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from amr_interfaces.action import ExecuteMission
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from amr_mission.mission_logic import (MissionError, MissionRunner, Outcome, Pose2D, RetryPolicy,
                                       load_stations, parse_mission)

_RESULT = {TaskResult.SUCCEEDED: Outcome.SUCCEEDED, TaskResult.CANCELED: Outcome.CANCELED}


# Executor implementation on top of Nav2 through nav2_simple_commander
class Nav2Executor:
    def __init__(self, nav: BasicNavigator, logger, step_timeout: float, dock_type: str,
                 docks: dict[str, Pose2D], staging_offset: float, set_docking_mode):
        self.nav, self.log = nav, logger
        self.step_timeout = step_timeout
        self.dock_type = dock_type
        self.docks = docks
        self.staging_offset = staging_offset
        self.set_docking_mode = set_docking_mode
        self.canceled = threading.Event()

    # Poll the running Nav2 task until it finishes, times out or is canceled
    def _wait_task(self) -> Outcome:
        start = time.monotonic()
        while not self.nav.isTaskComplete():
            if self.canceled.is_set():
                self.nav.cancelTask()
                return Outcome.CANCELED
            if time.monotonic() - start > self.step_timeout:
                self.log.warn("step timeout, canceling Nav2 task")
                self.nav.cancelTask()
                return Outcome.FAILED
            time.sleep(0.1)
        return _RESULT.get(self.nav.getResult(), Outcome.FAILED)

    def navigate(self, pose: Pose2D) -> Outcome:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = pose.x, pose.y
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
        if not self.nav.goToPose(goal):
            return Outcome.FAILED
        return self._wait_task()

    # Approach the staging pose with the normal safety fields, then dock with the docking field set
    def dock(self, dock_id: str) -> Outcome:
        if dock_id not in self.docks:
            self.log.error(f"unknown dock '{dock_id}'")
            return Outcome.FAILED
        d = self.docks[dock_id]
        staging = Pose2D(d.x - self.staging_offset * math.cos(d.yaw),
                         d.y - self.staging_offset * math.sin(d.yaw), d.yaw)
        outcome = self.navigate(staging)
        if outcome != Outcome.SUCCEEDED:
            return outcome
        self.set_docking_mode(True)
        try:
            time.sleep(0.5)
            self.nav.dockRobotByID(dock_id, nav_to_dock=False)
            return self._wait_task()
        finally:
            self.set_docking_mode(False)

    # The docking server aborts with success=true when the robot is not docked; nothing to undock counts as done
    def undock(self) -> Outcome:
        self.set_docking_mode(True)
        try:
            self.nav.undockRobot(self.dock_type)
            outcome = self._wait_task()
        finally:
            self.set_docking_mode(False)
        if outcome == Outcome.FAILED and self._last_result_success():
            self.log.info("robot was not docked, undock not needed")
            return Outcome.SUCCEEDED
        return outcome

    def _last_result_success(self) -> bool:
        future = getattr(self.nav, "result_future", None)
        if future is None or not future.done() or future.result() is None:
            return False
        return bool(getattr(future.result().result, "success", False))

    def wait(self, seconds: float) -> Outcome:
        if self.canceled.wait(timeout=seconds):
            return Outcome.CANCELED
        return Outcome.SUCCEEDED

    # Escalating recovery: clear costmaps first, additionally back up from the second attempt on
    def recover(self, attempt: int) -> None:
        self.log.warn(f"recovery #{attempt}: clearing costmaps")
        self.nav.clearAllCostmaps()
        if attempt >= 2:
            self.log.warn("recovery: backing up 0.3 m")
            self.nav.backup(backup_dist=0.3, backup_speed=0.1, time_allowance=10)
            self._wait_task()
        time.sleep(1.0)


# ExecuteMission action server; runs one mission at a time
class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_executor")
        default_stations = str(Path(get_package_share_directory("amr_mission")) / "config" / "stations.yaml")
        stations_file = self.declare_parameter("stations_file", default_stations).value
        self.max_attempts = self.declare_parameter("max_attempts", 3).value
        self.step_timeout = self.declare_parameter("step_timeout", 180.0).value
        self.dock_type = self.declare_parameter("dock_type", "simple_charging_dock").value
        self.stations = load_stations(yaml.safe_load(Path(stations_file).read_text()))
        self.get_logger().info(f"stations: {sorted(self.stations)}")

        # Dock poses from the docking database; the staging pose lies staging_offset in front of each dock
        default_docks = str(Path(get_package_share_directory("amr_navigation")) / "config" / "docks.yaml")
        docks_file = self.declare_parameter("docks_file", default_docks).value
        self.staging_offset = self.declare_parameter("staging_offset", 0.7).value
        docks = yaml.safe_load(Path(docks_file).read_text())["docks"]
        self.docks = {name: Pose2D(*map(float, d["pose"])) for name, d in docks.items()}
        self.docking_mode_pub = self.create_publisher(Bool, "safety/docking_mode", 10)

        # BasicNavigator is a separate node that spins itself while waiting for Nav2
        self.nav = BasicNavigator(node_name="mission_navigator")
        self.nav_ready = False
        self.busy = threading.Lock()
        self.runner: MissionRunner | None = None
        self.executor_impl: Nav2Executor | None = None

        self.server = ActionServer(
            self, ExecuteMission, "execute_mission", execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup())

    # Reject invalid missions and missions sent while another one is running
    def on_goal(self, goal):
        if self.busy.locked():
            self.get_logger().warn("rejecting mission: another mission is running")
            return GoalResponse.REJECT
        try:
            parse_mission(list(goal.steps), self.stations)
        except MissionError as exc:
            self.get_logger().error(f"rejecting mission: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def publish_docking_mode(self, on: bool):
        self.docking_mode_pub.publish(Bool(data=on))

    def on_cancel(self, _goal_handle):
        if self.runner:
            self.runner.cancel()
        if self.executor_impl:
            self.executor_impl.canceled.set()
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        with self.busy:
            req = goal_handle.request
            steps = parse_mission(list(req.steps), self.stations)
            if not self.nav_ready:
                self.get_logger().info("waiting for Nav2 to become active ...")
                self.nav.waitUntilNav2Active(localizer="amcl")
                self.nav_ready = True

            self.executor_impl = Nav2Executor(self.nav, self.get_logger(), self.step_timeout, self.dock_type,
                                              self.docks, self.staging_offset, self.publish_docking_mode)
            self.runner = MissionRunner(self.executor_impl, RetryPolicy(self.max_attempts))

            def feedback(i, step, status, attempt):
                fb = ExecuteMission.Feedback(step_index=i, current_step=step.raw, status=status, attempt=attempt)
                goal_handle.publish_feedback(fb)
                self.get_logger().info(f"[{req.mission_id}] step {i} {step.raw}: {status} (attempt {attempt})")

            res = self.runner.run(steps, feedback)
            result = ExecuteMission.Result(success=res.success, message=res.message,
                                           failed_step=res.failed_step, duration_s=float(res.duration_s))
            if res.success:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().info(f"[{req.mission_id}] {res.message} in {res.duration_s:.1f} s")
            self.runner = None
            return result


def main():
    rclpy.init()
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
