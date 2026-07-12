"""PGuard + Nav2 with a static Novation City map (outdoor, EKF localizer).

Adds two things missing from full_stack.launch.py so that navigate_to_pose
actually plans:
  1. nav2_map_server publishing /map from maps/novation_city.yaml
  2. static_transform_publisher map->odom (identity - EKF datum is at (0,0))
  3. robo_fleet_adapter for /pguard/* namespace + rosbridge port 9090

Launch alone (assumes sim.launch.py + localization.launch.py + Nav2 already
running via full_stack.launch.py).
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('my_pguard_bot')
    map_yaml = PathJoinSubstitution([pkg, 'maps', 'novation_city.yaml'])

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml, 'topic_name': 'map'}],
        output='screen',
    )

    # Add map_server to the lifecycle manager (it was not in the original list).
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}],
    )

    # Static map->odom transform (identity - the EKF datum IS the map origin).
    tf_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    adapter = Node(
        package='my_pguard_bot', executable='robo_fleet_adapter',
        name='robo_fleet_adapter', output='screen',
    )

    # rosbridge websocket on 9090 for robo_fleet MCP server.
    rosbridge = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen',
        parameters=[{'port': 9090}],
    )

    return LaunchDescription([map_server, lifecycle_mgr, tf_map_odom, adapter, rosbridge])
