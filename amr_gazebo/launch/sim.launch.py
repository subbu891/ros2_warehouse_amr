"""Start Gazebo Harmonic with the warehouse, spawn the AMR and start its controllers.

    ros2 launch amr_gazebo sim.launch.py               # with Gazebo GUI
    ros2 launch amr_gazebo sim.launch.py headless:=true # server only (CI, low-RAM machines)
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("amr_description")
    gz_pkg = FindPackageShare("amr_gazebo")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    x, y, yaw = LaunchConfiguration("x"), LaunchConfiguration("y"), LaunchConfiguration("yaw")
    use_camera = LaunchConfiguration("use_camera")

    controllers = PathJoinSubstitution([desc_pkg, "config", "amr_controllers.yaml"])
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([desc_pkg, "urdf", "amr.urdf.xacro"]),
            " use_sim:=true use_camera:=", use_camera,
            " controllers_file:=", controllers,
        ]),
        value_type=str,
    )

    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r ", world], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-s -r --headless-rendering ", world],
                          "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "amr",
                   "-x", x, "-y", y, "-z", "0.02", "-Y", yaw],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": PathJoinSubstitution([gz_pkg, "config", "gz_bridge.yaml"]),
                     "use_sim_time": True}],
    )

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    ddc = Node(package="controller_manager", executable="spawner",
               arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([gz_pkg, "worlds", "warehouse.sdf"])),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument("x", default_value="2.0"),
        DeclareLaunchArgument("y", default_value="4.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        gazebo_gui, gazebo_headless, bridge, rsp, spawn,
        # Controllers can only load once the robot (and its gz_ros2_control plugin) exists
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, ddc])),
    ])
