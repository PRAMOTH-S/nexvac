from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():


    description_package = get_package_share_directory(
        'nexva_description'
    )



    xacro_file = os.path.join(
        description_package,
        'urdf',
        'nexva.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {
        'robot_description': robot_description_config.toxml()
    }



    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    firmware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nexva_frimware'),
                'launch',
                'robot.launch.py'
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    # BNO055 + stall_guard. Safe to include unconditionally: if the board is
    # not wired, or the adafruit libraries are not installed, the driver logs
    # exactly what is missing and keeps running without publishing, and the
    # guard reports "NO IMU - guard inactive" and never fires. Nothing here
    # can fail bringup. `imu:=false` skips both nodes entirely.
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nexva_sensor'),
                'launch',
                'imu.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('imu')),
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'imu', default_value='true',
            description='Start the BNO055 driver and the stall guard. An '
                        'absent board is logged and ignored, so this only '
                        'needs turning off to silence the warnings'),

        robot_state_publisher,

        firmware_launch,

        lidar_launch,

        imu_launch,

    ])