"""PGuard full stack (PearlGuard robot, TWO robots): sim + dual-EKF
localization + Nav2, each robot fully namespaced.

Pass `use_gui:=true` to launch the Gazebo GUI (needs display + GPU).
Pass `rviz:=true` to also start RViz2 with the Nav2 panel.

Requires:
  - pguard_x.gazebo.xacro / VLP-16.urdf.xacro patched to accept a
    `namespace` xacro arg and prefix every <topic> with it.
  - nav2_params.yaml: obstacle_layer indentation fixed, topics made
    relative (no leading '/').
  - ekf.yaml: topics made relative too (odom0: odom, not /odom, etc.)
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
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from os import name, pathsep

ROBOTS = [
    {"name": "pearlguard1", "x": 0.0,  "y": 0.0},
    #{"name": "pearlguard2", "x": 15.0, "y": 0.0},
]

NAV2_LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    pkg = FindPackageShare('pearlguard_description')
    pkg_pguard = FindPackageShare('my_pguard_bot')  
    pkg_description = get_package_share_directory('pearlguard_description')

    model_path = str(Path(pkg_description).parent.resolve())
    model_path += pathsep + str(
        Path(pkg_description).parent.parent.parent.parent.resolve() / "src"
    )
    model_path += pathsep + pkg_description

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path
    )
    xacro_file = PathJoinSubstitution([pkg, 'pguard.xacro'])
    world_file = PathJoinSubstitution([pkg_pguard, 'worlds', 'novation_city.sdf'])   # shared world
    nav2_params = PathJoinSubstitution([pkg_pguard, 'config', 'nav2_params.yaml'])
    map_yaml = PathJoinSubstitution([pkg_pguard, 'maps', 'novation_city.yaml'])      # shared
    rviz_cfg = PathJoinSubstitution([pkg_pguard, 'rviz', 'pguard_nav2.rviz']) 
    gui_arg = DeclareLaunchArgument('use_gui', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    # ─── Gazebo server / GUI (shared, one world for both robots) ───
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '3', world_file],
        output='screen',
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    # ─── Shared static map (one map, one map_server, no namespace) ───
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml, 'topic_name': 'map'}],
        output='screen',
    )
    map_lifecycle_mgr = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}],
    )

    def make_sim_group(name: str, x: float, y: float, z: float = 0.4):
        """robot_state_publisher + delayed spawn + this robot's bridge."""
        robot_description = {
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' namespace:=', name]),
                value_type=str,
            )
        }
        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}],
        )
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[ '-name', name,
                        '-topic',f'{name}/robot_description',
                        '-x', str(x),
                        '-y', str(y),
                        '-z', str(z)],
        )
        delayed_spawn = RegisterEventHandler(
            OnProcessStart(target_action=gz_server,
                            on_start=[TimerAction(period=3.0, actions=[spawn])])
        )
        bridge_yaml = PathJoinSubstitution([pkg_pguard, 'config', f'bridge_{name}.yaml'])
        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        )
        return GroupAction([PushRosNamespace(name), rsp, delayed_spawn, bridge])

    def make_localization_group(name: str):
        """Dual-EKF + navsat_transform for one robot, using this robot's own
        fully pre-written ekf_<name>.yaml (frames/topics already namespaced
        in the file itself, since RewrittenYaml can't scope sibling keys
        like ekf_local/ekf_global that share parameter names)."""
        ekf_config = PathJoinSubstitution(
            [FindPackageShare('my_pguard_bot'), 'config', f'ekf_{name}.yaml']
        )

        ekf_local = Node(
            package='robot_localization', executable='ekf_node', name='ekf_local',
            output='screen', parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/filtered_local')],
        )
        ekf_global = Node(
            package='robot_localization', executable='ekf_node', name='ekf_global',
            output='screen', parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odometry/filtered')],
        )
        navsat = Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen', parameters=[ekf_config],
            remappings=[
                ('imu', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/filtered'),
            ],
    )
        gps_tf = Node(
            package='my_pguard_bot', executable='gps_tf_publisher',
            output='screen',
        )
        return GroupAction([PushRosNamespace(name), ekf_local, ekf_global, navsat, gps_tf])

    def make_nav2_group(name: str):
        """Full Nav2 stack inside a component container (composition mode).
        This is REQUIRED in Jazzy because standalone --params-file does not
        load undeclared plugin parameters (e.g. FollowPath.plugin, critics)."""
        params_path = PathJoinSubstitution(
            [FindPackageShare('my_pguard_bot'), 'config', f'nav2_params_{name}.yaml']
        )
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_path,
                param_rewrites={},
                root_key=name,
                convert_types=True,
            ),
            allow_substs=True,
        )
        container_name = f'nav2_container_{name}'
        nav2_group = GroupAction([
            PushRosNamespace(name),
            Node(
                package='rclcpp_components',
                executable='component_container_isolated',
                name=container_name,
                parameters=[configured_params],
                output='screen',
            ),
            LoadComposableNodes(
                target_container=f'{name}/{container_name}',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=[
                            ('navigate_to_pose', f'/{name}/navigate_to_pose'),
                            ('navigate_through_poses', f'/{name}/navigate_through_poses'),
                        ],
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[{'use_sim_time': True, 'autostart': True,
                                    'node_names': NAV2_LIFECYCLE_NODES}],
                    ),
                ],
            ),
        ])
        return TimerAction(period=6.0, actions=[nav2_group])
    # ─── Build both robots ───
    sim_groups = [make_sim_group(r['name'], r['x'], r['y']) for r in ROBOTS]
    localization_groups = [
        TimerAction(period=3.0, actions=[make_localization_group(r['name'])])
        for r in ROBOTS
    ]
    nav2_groups = [make_nav2_group(r['name']) for r in ROBOTS]

    # ─── Optional RViz (one instance is enough to view both via TF tree) ───
    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
    [
        gui_arg,
        rviz_arg,
        gz_resource_path,
        gz_server,
        gz_gui,
        map_server,
        map_lifecycle_mgr,
    ]
    + sim_groups
    + localization_groups
    + nav2_groups
    + [rviz_node]
)