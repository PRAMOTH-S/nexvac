import os
import shutil
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

# Pulse the ESP32's auto-reset line (RTS -> EN) so it boots into a fresh
# micro-ROS session before the agent opens the port. Without this, a board left
# over from a previous run keeps pinging on its old session id and the new
# agent ignores it.
RESET_ESP = (
    "import serial, time; "
    "s = serial.Serial('/dev/esp', 460800); "
    "s.dtr = False; s.rts = True; time.sleep(0.15); "
    "s.rts = False; time.sleep(0.05); s.close()"
)


def generate_launch_description():

    # Clear stale agents, then reset the board, before the new agent starts.
    # SIGCONT first: an agent suspended with Ctrl+Z cannot act on SIGTERM.
    # SIGTERM (not SIGKILL) lets it close the session cleanly; SIGKILL is the
    # last resort. The [m] stops pkill -f from matching this command itself.
    cleanup_processes = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -CONT -f "[m]icro_ros_agent" 2>/dev/null; '
             'pkill -TERM -f "[m]icro_ros_agent" 2>/dev/null; '
             'sleep 1; '
             'pkill -KILL -f "[m]icro_ros_agent" 2>/dev/null; '
             'python3 -c "{}" || echo "ESP32 reset skipped"; '
             'exit 0'.format(RESET_ESP)],
        name='cleanup_processes',
        output='screen',
    )

    # Pinned to one core, and it is the ONLY thing here that is pinned.
    #
    # The kernel already spreads the other nodes across the Pi 5's four cores
    # and does it better than a fixed map would; pinning everything would just
    # serialise work onto fewer cores and run slower. This process is the
    # exception: it is a 460800-baud serial link with a 500 ms watchdog at the
    # far end, so being preempted mid-transfer means the ESP32 stops hearing
    # cmd_vel and the wheels stop. A core it does not have to contend for
    # removes that failure mode.
    #
    # NEXVA_AGENT_CORE=none turns it off; taskset missing is not fatal.
    # launch_ros wants prefix as a SHELL STRING, not a list. Passing a list
    # makes the executor try to exec 'taskset' with the rest swallowed, and it
    # dies with "exception occurred while executing process" before the agent
    # ever opens the serial port - i.e. no /odom, and bringup never comes up.
    agent_core = os.environ.get('NEXVA_AGENT_CORE', '3')
    agent_prefix = ''
    if agent_core != 'none' and shutil.which('taskset'):
        agent_prefix = 'taskset -c %s' % agent_core

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        prefix=agent_prefix,
        arguments=[
            'serial',
            '--dev', '/dev/esp',
            '-b', '460800'
        ],
        emulate_tty=True,
    )

    # Encoder counts -> /joint_states, so robot_state_publisher can place the
    # wheel links. Derived on this side to keep the serial link free.
    wheel_joint_publisher = Node(
        package='nexva_frimware',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher',
        output='screen',
    )

    # Encoder counts -> /odom and odom -> base_footprint. Integrated here
    # rather than on the ESP32 because nav_msgs/Odometry exceeds the serial
    # MTU and stalls the reliable stream to 1 Hz.
    wheel_odometry = Node(
        package='nexva_frimware',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
    )

    # Start the agent only once cleanup has actually exited. Listing both
    # actions in the LaunchDescription would start them in parallel, letting
    # the cleanup race the agent it is meant to prepare for.
    start_agent_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_processes,
            on_exit=[
                LogInfo(msg="Port clear, ESP32 reset. Starting Micro-ROS Agent."),
                micro_ros_agent,
                wheel_joint_publisher,
                wheel_odometry,
            ],
        )
    )

    return LaunchDescription([
        cleanup_processes,
        start_agent_after_cleanup,
    ])
