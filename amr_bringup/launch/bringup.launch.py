from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(pkg, file, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", file])),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("amr_bringup"), "config", "amr_params.yaml"])
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    forklift = LaunchConfiguration("forklift")
    vda = LaunchConfiguration("vda5050")
    nav_delay = LaunchConfiguration("nav_delay")

    def py_node(pkg, exe, name):
        return Node(package=pkg, executable=exe, name=name, parameters=[params], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("forklift", default_value="true"),
        DeclareLaunchArgument("vda5050", default_value="false"),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare("amr_navigation"), "maps", "warehouse.yaml"])),
        DeclareLaunchArgument("nav_delay", default_value="8.0"),

        # Simulation and the safety monitor start first
        include("amr_gazebo", "sim.launch.py", {"headless": headless}),
        py_node("amr_safety", "safety_monitor", "safety_monitor"),

        # Localization, navigation and perception wait until Gazebo and the controllers are up
        TimerAction(period=nav_delay, actions=[
            include("amr_navigation", "localization.launch.py", {"map": LaunchConfiguration("map")}),
            include("amr_navigation", "navigation.launch.py"),
            py_node("amr_perception", "obstacle_tracker", "obstacle_tracker"),
            py_node("amr_mission", "mission_executor", "mission_executor"),
            Node(package="amr_validation", executable="forklift_mover", name="forklift_mover",
                 parameters=[params, {"enabled": ParameterValue(forklift, value_type=bool)}],
                 output="screen"),
            # Fleet interface, needs a running MQTT broker
            Node(package="amr_vda5050", executable="vda5050_adapter", name="vda5050_adapter",
                 parameters=[params], output="screen", condition=IfCondition(vda)),
        ]),

        Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(rviz),
             arguments=["-d", PathJoinSubstitution([FindPackageShare("amr_description"), "rviz", "amr.rviz"])],
             parameters=[{"use_sim_time": True}]),
    ])
