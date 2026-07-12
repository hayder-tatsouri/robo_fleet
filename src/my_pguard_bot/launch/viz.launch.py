"""PGuard visualization stack: Foxglove Bridge for browser-based viewing.

Serves ROS 2 topics over WebSocket on port 8765. Connect from a browser at
https://studio.foxglove.dev  ->  Open connection  ->  Foxglove WebSocket
   ws://<host-ip>:8765
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'use_sim_time': True,
            'capabilities': ['clientPublish', 'parameters',
                             'parametersSubscribe', 'services',
                             'connectionGraph', 'assets'],
            'send_buffer_limit': 10000000,
        }],
    )
    return LaunchDescription([foxglove])
