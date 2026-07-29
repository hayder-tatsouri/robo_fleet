import tempfile

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    use_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = xacro.process_file(model_path).toxml()
    joint_state_description = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='pearlguard_description_',
        suffix='.urdf',
        delete=False,
    )
    joint_state_description.write(robot_description)
    joint_state_description.close()

    return [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            arguments=[joint_state_description.name],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            arguments=[joint_state_description.name],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
            ],
        ),
    ]


def generate_launch_description():
    package_name = 'pearlguard_description'
    default_model = PathJoinSubstitution([FindPackageShare(package_name), 'pguard.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot xacro file.',
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='false',
            description='Start joint_state_publisher_gui instead of joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock if true.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
