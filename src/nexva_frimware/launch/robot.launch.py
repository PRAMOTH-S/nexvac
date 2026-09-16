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
    "s = serial.Serial('/dev/esp', 115200); "
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

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', '/dev/esp',
            '-b', '115200'
        ],
        emulate_tty=True,
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
            ],
        )
    )

    return LaunchDescription([
        cleanup_processes,
        start_agent_after_cleanup,
    ])
