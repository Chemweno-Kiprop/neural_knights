import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'neural_knights'  # <--- your package

    # Robot State Publisher (publishes robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Path to robot description and controllers
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    controllers_file = os.path.join(pkg_path, "config", "my_controllers.yaml")

    # Process xacro into robot_description string
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    # ros2_control node (controller_manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file
        ],
        output="screen"
    )

    # Launch Ignition Gazebo Fortress with empty.sdf
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # Spawner nodes for ros2_control controllers
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/controller_manager"],
        output="screen"
    )

    # Delay spawners until ros2_control_node is ready
    delayed_joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[joint_broad_spawner]
    )

    delayed_diff_drive_spawner = TimerAction(
        period=7.0,
        actions=[diff_drive_spawner]
    )

    # Spawner node for robot entity (Ignition create)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_bot"
        ],
        output="screen"
    )

    # Delay spawner to wait for Ignition to be ready
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        rsp,
        control_node,
        ign_gazebo,
        delayed_spawn,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner
    ])
