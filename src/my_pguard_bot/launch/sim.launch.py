"""Launch PGuard sim (PearlGuard robot): Gazebo + RSP + spawn + ros_gz_bridge,
for TWO robots, fully namespaced.

By default runs Gazebo in headless server-only mode (`gz sim -s`). Pass
`use_gui:=true` to also start the Gazebo GUI on a GPU-capable host.

Requires: pguard_x.gazebo.xacro and VLP-16.urdf.xacro patched to accept
a `namespace` xacro arg and prefix every <topic> with it. Without that
patch, both robots will collide on identical topic names.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from pathlib import Path
from os import pathsep


def generate_launch_description():
    pkg = FindPackageShare('pearlguard_description')
    pkg_description_path = str(pkg)
    pkg_pguard = FindPackageShare('my_pguard_bot')   # <-- add this
    xacro_file = PathJoinSubstitution([pkg, 'pguard.xacro'])
    world_file = PathJoinSubstitution([pkg_pguard, 'worlds', 'novation_city.sdf'])
    model_path = str(Path(pkg_description_path).parent.resolve())
    model_path += pathsep + str(Path(pkg_description_path).parent.parent.parent.parent.resolve() / "src")
    model_path += pathsep + pkg_description_path

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )
    
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

    def make_robot_group(name: str, x: float, y: float, z: float = 0.4):
        """One fully namespaced robot: robot_state_publisher + delayed spawn
        + its own ros_gz_bridge instance."""
        bridge_yaml = PathJoinSubstitution([pkg_pguard, 'config', f'bridge_{name}.yaml'])
        robot_description = {
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' namespace:=', name]),
                value_type=str,
            )
        }

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description,
                        {'use_sim_time': True, 'frame_prefix': f'{name}/'}],
        )

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', name,
                '-topic', f'/{name}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
            ],
        )

        delayed_spawn = RegisterEventHandler(
            OnProcessStart(
                target_action=gz_server,
                on_start=[TimerAction(period=3.0, actions=[spawn])]
            )
        )

        bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        )

        return GroupAction([
            PushRosNamespace(name),
            rsp,
            delayed_spawn,
            bridge,
        ])

    # Two robots, spawned apart so they don't overlap.
    robot1 = make_robot_group('pearlguard1', x=0.0,  y=0.0)
    robot2 = make_robot_group('pearlguard2', x=15.0, y=0.0)

    return LaunchDescription(
    [
        gui_arg,
        gz_resource_path,
        gz_server,
        gz_gui,
        robot1,
        robot2
    ]
)