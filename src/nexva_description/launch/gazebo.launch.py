
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "nexva_description"

    pkg_nexva_description = get_package_share_directory(
        package_name
    )

    # ============================================================
    # WORLD FILE
    # ============================================================

    world_file = os.path.join(
        pkg_nexva_description,
        "worlds",
        "vacuum_world.sdf"
    )

    # ============================================================
    # GAZEBO RESOURCE PATH
    # ============================================================

    gazebo_models_path = os.path.dirname(
        pkg_nexva_description
    )

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Open RViz"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time"
    )

    # ============================================================
    # NEXVA XACRO FILE
    # ============================================================

    urdf_file_path = PathJoinSubstitution([
        pkg_nexva_description,
        "urdf",
        "nexva.urdf.xacro"
    ])

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node =  Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]

    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items()
    )

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",

        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([
                        "xacro",
                        " ",
                        urdf_file_path
                    ]),
                    value_type=str
                ),

                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ]
    )

    # ============================================================
    # SPAWN NEXVA INTO GAZEBO
    # ============================================================

    spawn_nexva = Node(
        package="ros_gz_sim",
        executable="create",

        arguments=[
            "-name",
            "nexva",

            "-topic",
            "robot_description",

            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.0",

            "-Y",
            "1.5708",
        ],

        output="screen"
    )

    # ============================================================
    # RVIZ
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",

        arguments=[
            "-d",
            os.path.join(
                pkg_nexva_description,
                "rviz",
                "rviz.rviz"
            )
        ],

        condition=IfCondition(
            LaunchConfiguration("rviz")
        ),

        parameters=[
            {
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                )
            }
        ],

        output="screen"
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        rviz_arg,

        use_sim_time_arg,

        # Gazebo + vacuum world
        gazebo,

        # Publish Nexva robot description and TF
        robot_state_publisher,

        # Spawn Nexva into Gazebo
        spawn_nexva,

        # RViz
        rviz,
        gz_bridge_node,
    ])


