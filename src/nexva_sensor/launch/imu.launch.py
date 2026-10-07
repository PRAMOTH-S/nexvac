"""
BNO055 IMU, accelerometer only, plus the motion check that reads it.

WHAT CHANGED AND WHY

This used to start `stall_guard` and `odom_guard` as well. Both are gone:

  - `stall_guard` raised the ESP32's PWM floor through /pid_limits to break a
    suspected stall, and restored it on recovery. Killed before it restored,
    it left the firmware with a raised floor and the robot drove itself with
    NOTHING publishing cmd_vel. That is a dangerous failure mode for a
    convenience feature, and it is why nothing here writes to /pid_limits now.
  - `odom_guard` froze odometry on the stall verdict. A false verdict then
    stopped the pose updating while the robot was genuinely driving, which is
    worse than the phantom motion it existed to remove.

What replaces them is `motion_check`: accelerometer in, a verdict out, and no
authority over anything. It publishes `motion_check/moved` (Bool) and
`motion_check/status` (String). Nothing acts on it automatically - read it in
the web UI, or subscribe to it deliberately.

ACCELEROMETER ONLY

`accel_only` (default true) publishes linear acceleration and angular velocity
but marks orientation unavailable per REP-145, and skips the magnetometer. An
uncalibrated BNO055 reports a fused heading that is confidently wrong, so
nothing downstream is allowed to trust it. `accel_only:=false` restores the
full fused output once the chip is calibrated.

BEFORE THIS IS WORTH ANYTHING

Measured on this robot: 16.4% of I2C reads failing (Errno 121), accelerometer
calibration 0/3, and resting noise (0.126-0.150 m/s2) indistinguishable from
driving at 0.035 m/s. Re-seat the four wires - especially GND to pin 6 - and
calibrate before reading anything into the numbers. `tools/check_imu.sh` walks
the whole chain.
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable = LaunchConfiguration('enable')
    accel_only = LaunchConfiguration('accel_only')
    motion_check = LaunchConfiguration('motion_check')
    frame_id = LaunchConfiguration('frame_id')
    rate = LaunchConfiguration('rate')
    address = LaunchConfiguration('address')
    axis_remap = LaunchConfiguration('axis_remap')
    min_calibration = LaunchConfiguration('min_calibration')

    args = [
        DeclareLaunchArgument(
            'enable', default_value='true',
            description='Start the BNO055 driver'),
        DeclareLaunchArgument(
            'accel_only', default_value='true',
            description='Publish acceleration and angular velocity, but mark '
                        'orientation unusable and skip the magnetometer'),
        DeclareLaunchArgument(
            'motion_check', default_value='true',
            description='Start the accelerometer-only "has it moved" check. '
                        'Reports only - it never writes cmd_vel or pid_limits'),
        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description='Frame to stamp the IMU with. base_link because the '
                        'URDF has no imu_link yet'),
        DeclareLaunchArgument(
            'rate', default_value='50.0',
            description='Publish rate in Hz; the Pi I2C bus is the real limit'),
        DeclareLaunchArgument(
            'address', default_value='40',
            description='I2C address, decimal. 40 = 0x28, 41 = 0x29'),
        DeclareLaunchArgument(
            'axis_remap', default_value='[1, 2, 3]',
            description='Signed permutation mapping chip axes to REP-103. '
                        'Verify on the bench: turning left must raise yaw'),
        DeclareLaunchArgument(
            'min_calibration', default_value='2',
            description='Calibration level (0-3) below which the fused '
                        'heading is published as untrusted'),
    ]

    imu = Node(
        package='nexva_sensor',
        executable='bno055_imu',
        name='bno055_imu',
        output='screen',
        condition=IfCondition(enable),
        parameters=[{
            'use_sim_time': False,
            'accel_only': ParameterValue(accel_only, value_type=bool),
            'frame_id': frame_id,
            'rate': ParameterValue(rate, value_type=float),
            'address': ParameterValue(address, value_type=int),
            'axis_remap': ParameterValue(axis_remap, value_type=List[int]),
            'min_calibration': ParameterValue(min_calibration, value_type=int),
        }],
    )

    check = Node(
        package='nexva_sensor',
        executable='motion_check',
        name='motion_check',
        output='screen',
        condition=IfCondition(motion_check),
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription(args + [imu, check])
