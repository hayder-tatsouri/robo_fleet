"""Dual-EKF outdoor localization for PGuard (odom+IMU+RTK GPS)."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('my_pguard_bot')
    ekf_config = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odometry/filtered_local')],
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([ekf_local, ekf_global, navsat])
