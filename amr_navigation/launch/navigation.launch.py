from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NAV_NODES = [
    "controller_server", "smoother_server", "planner_server", "behavior_server",
    "velocity_smoother", "bt_navigator", "waypoint_follower", "docking_server",
]
FILTER_NODES = [
    "keepout_filter_mask_server", "keepout_costmap_filter_info_server",
    "speed_filter_mask_server", "speed_costmap_filter_info_server",
]


def generate_launch_description():
    pkg = FindPackageShare("amr_navigation")
    params = LaunchConfiguration("params_file")

    # Every Nav2 velocity output goes to cmd_vel_nav; only the safety monitor drives the wheels
    to_nav = [("cmd_vel", "cmd_vel_nav")]

    def nav2(package, executable, name=None, remaps=None, extra=None):
        return Node(package=package, executable=executable, name=name or executable,
                    output="screen", parameters=[params] + ([extra] if extra else []),
                    remappings=remaps or [])

    # Keepout and speed zone masks with their filter info servers
    filters = [
        nav2("nav2_map_server", "map_server", "keepout_filter_mask_server",
             extra={"yaml_filename": PathJoinSubstitution([pkg, "maps", "keepout_mask.yaml"])}),
        nav2("nav2_map_server", "costmap_filter_info_server", "keepout_costmap_filter_info_server"),
        nav2("nav2_map_server", "map_server", "speed_filter_mask_server",
             extra={"yaml_filename": PathJoinSubstitution([pkg, "maps", "speed_mask.yaml"])}),
        nav2("nav2_map_server", "costmap_filter_info_server", "speed_costmap_filter_info_server"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_filters", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True, "node_names": FILTER_NODES}]),
    ]

    # Planning, control, recovery, smoothing and docking servers
    servers = [
        nav2("nav2_controller", "controller_server", remaps=to_nav),
        nav2("nav2_smoother", "smoother_server"),
        nav2("nav2_planner", "planner_server"),
        nav2("nav2_behaviors", "behavior_server", remaps=to_nav),
        nav2("nav2_velocity_smoother", "velocity_smoother",
             remaps=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel_smoothed")]),
        nav2("nav2_bt_navigator", "bt_navigator"),
        nav2("nav2_waypoint_follower", "waypoint_follower"),
        nav2("opennav_docking", "opennav_docking", "docking_server", remaps=to_nav,
             extra={"dock_database": PathJoinSubstitution([pkg, "config", "docks.yaml"])}),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 10.0,
                          "node_names": NAV_NODES}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])),
        *filters,
        *servers,
    ])
