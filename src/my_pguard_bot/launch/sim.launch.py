"""Launch PGuard sim: Gazebo + RSP + spawn + ros_gz_bridge.

By default runs Gazebo in headless server-only mode (`gz sim -s`). Pass
`use_gui:=true` to also start the Gazebo GUI on a GPU-capable host.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('my_pguard_bot')

    xacro_file = PathJoinSubstitution([pkg, 'description', 'pguard.urdf.xacro'])
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

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

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
