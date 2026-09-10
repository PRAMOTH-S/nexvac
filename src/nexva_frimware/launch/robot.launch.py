from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', '/dev/esp',
            '-b', '115200'
        ]
    )

    return LaunchDescription([
        micro_ros_agent
    ])