"""PGuard full stack: sim + dual-EKF localization + Nav2.

Pass `use_gui:=true` to launch the Gazebo GUI (needs display + GPU).
Pass `rviz:=true` to also start RViz2 with the Nav2 panel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('my_pguard_bot')
    nav2_params = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])

    gui_arg = DeclareLaunchArgument('use_gui', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])
        ),
        launch_arguments={'use_gui': LaunchConfiguration('use_gui')}.items(),
    )

    viz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg, 'launch', 'viz.launch.py'])
    ))

    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'pguard_nav2.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    localization = TimerAction(period=1.5, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'localization.launch.py'])
        ))
    ])

    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
        'velocity_smoother',
    ]

    nav2 = TimerAction(period=3.0, actions=[
        Node(package='nav2_controller',        executable='controller_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_smoother',          executable='smoother_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_planner',           executable='planner_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_behaviors',         executable='behavior_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'node_names': lifecycle_nodes}]),
    ])

    return LaunchDescription([gui_arg, rviz_arg, sim, localization, nav2, viz, rviz_node])
