from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("amr_description")
    use_camera = LaunchConfiguration("use_camera")

    # Robot description from xacro, without simulation plugins
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([pkg, "urdf", "amr.urdf.xacro"]),
            " use_sim:=false use_camera:=", use_camera,
        ]),
        value_type=str,
    )

    # Model viewer: state publisher, joint sliders and RViz
    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "amr.rviz"]), "-f", "base_footprint"]),
    ])
