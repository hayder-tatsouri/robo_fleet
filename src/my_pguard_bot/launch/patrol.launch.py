"""PGuard perimeter patrol launcher.

Runs the GPS-waypoint patrol client against a running Nav2 stack (assumed
already up, e.g. via full_stack.launch.py). Handy so you can `Ctrl-C` the
patrol independently and restart it without touching the sim.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("my_pguard_bot")
    default_yaml = PathJoinSubstitution([pkg, "config", "patrol_waypoints.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("yaml_file",     default_value=default_yaml),
        DeclareLaunchArgument("loop",          default_value="true"),
        DeclareLaunchArgument("loop_delay_s",  default_value="5.0"),
        DeclareLaunchArgument("start_delay_s", default_value="8.0"),

        Node(
            package="my_pguard_bot",
            executable="patrol_client",
            name="pguard_patrol_client",
            output="screen",
            parameters=[{
                "yaml_file":     LaunchConfiguration("yaml_file"),
                "loop":          LaunchConfiguration("loop"),
                "loop_delay_s":  LaunchConfiguration("loop_delay_s"),
                "start_delay_s": LaunchConfiguration("start_delay_s"),
                "use_sim_time":  True,
            }],
        ),
    ])
