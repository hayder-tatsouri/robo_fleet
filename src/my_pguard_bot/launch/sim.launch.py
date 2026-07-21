"""Launch PGuard sim: Gazebo + RSP + spawn + ros_gz_bridge.

By default runs Gazebo in headless server-only mode (`gz sim -s`). Pass
`use_gui:=true` to also start the Gazebo GUI on a GPU-capable host.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cached_urdf() -> str:
    """Expand pguard.urdf.xacro once per install and cache the result.

    Re-running xacro on every launch adds 1-2 s of subprocess overhead and
    is redundant when the xacro file hasn't changed. We stamp the xacro's
    mtime into the cache filename so an edit invalidates the cache
    automatically.
    """
    pkg_share = get_package_share_directory('my_pguard_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'pguard.urdf.xacro')
    mtime = int(os.path.getmtime(xacro_file))
    cache_dir = os.path.expanduser('~/.cache/pguard')
    os.makedirs(cache_dir, exist_ok=True)
    cache_path = os.path.join(cache_dir, f'pguard.{mtime}.urdf')
    if not os.path.exists(cache_path):
        urdf = subprocess.check_output(['xacro', xacro_file], text=True)
        with open(cache_path, 'w') as fh:
            fh.write(urdf)
    return cache_path


def generate_launch_description():
    pkg = FindPackageShare('my_pguard_bot')

    world_file = PathJoinSubstitution([pkg, 'worlds', 'novation_city.sdf'])
    bridge_yaml = PathJoinSubstitution([pkg, 'config', 'bridge.yaml'])

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.4')
    gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Set to true to launch the Gazebo GUI (requires a display + GPU).'
    )

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '3', world_file],
        output='screen',
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    urdf_path = _cached_urdf()
    with open(urdf_path) as fh:
        robot_description = {'robot_description': fh.read()}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'pguard',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
    )

    return LaunchDescription(
        [x_arg, y_arg, z_arg, gui_arg, gz_server, gz_gui, rsp, spawn, bridge]
    )
