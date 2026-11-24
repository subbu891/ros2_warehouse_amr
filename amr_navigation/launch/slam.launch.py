from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("amr_navigation")
    return LaunchDescription([
        # Sensor fusion: wheel odometry + IMU -> odom -> base_footprint
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[PathJoinSubstitution([pkg, "config", "ekf.yaml"])], output="screen"),
        # Online mapping: lidar scan matching -> map -> odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])),
            launch_arguments={
                "slam_params_file": PathJoinSubstitution([pkg, "config", "slam_toolbox.yaml"]),
                "use_sim_time": "true",
            }.items(),
        ),
    ])
