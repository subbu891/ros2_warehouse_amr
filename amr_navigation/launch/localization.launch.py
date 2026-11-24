from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("amr_navigation")
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution([pkg, "maps", "warehouse.yaml"])),

        # Sensor fusion: odom -> base_footprint
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[PathJoinSubstitution([pkg, "config", "ekf.yaml"])], output="screen"),

        # Known map and particle filter localization: map -> odom
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[params, {"yaml_filename": map_yaml}], output="screen"),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             parameters=[params], output="screen"),

        # Configures and activates the lifecycle nodes above
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server", "amcl"]}]),
    ])
