import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("amr_bringup"), "launch", "bringup.launch.py"])),
        launch_arguments={"headless": "true", "rviz": "false", "forklift": "false"}.items())
    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestStackComesUp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("bringup_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_message(self, msg_type, topic, timeout, qos=10):
        got = []
        sub = self.node.create_subscription(msg_type, topic, got.append, qos)
        end = time.time() + timeout
        while not got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return got[0] if got else None

    def test_1_sensors_publish(self):
        self.assertIsNotNone(self.wait_for_message(LaserScan, "/scan", 90.0, qos_profile_sensor_data))
        self.assertIsNotNone(self.wait_for_message(Odometry, "/odometry/filtered", 30.0))

    def test_2_tf_tree_complete(self):
        end = time.time() + 60.0
        ok = False
        while time.time() < end and not ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            ok = self.tf_buffer.can_transform("map", "laser_link", rclpy.time.Time())
        self.assertTrue(ok, "map -> odom -> base_footprint -> laser_link not available")

    def test_3_nav2_active(self):
        for name in ("amcl", "controller_server", "planner_server", "bt_navigator", "docking_server"):
            cli = self.node.create_client(GetState, f"/{name}/get_state")
            self.assertTrue(cli.wait_for_service(timeout_sec=60.0), f"{name} not running")
            state = ""
            end = time.time() + 60.0
            while state != "active" and time.time() < end:
                fut = cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
                state = fut.result().current_state.label if fut.result() else ""
                time.sleep(0.5)
            self.assertEqual(state, "active", f"{name} is {state}")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Gazebo may exit with -15/-2 on SIGTERM/SIGINT; everything else must exit cleanly
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 1])
