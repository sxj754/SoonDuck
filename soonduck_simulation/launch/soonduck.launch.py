import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory("soonduck_simulation"),
        "worlds",
        "obstacles.world",
    )
    gazebo_params_path = os.path.join(
        get_package_share_directory("soonduck_simulation"),
        "config",
        "gazebo_params.yaml",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ),
            ]
        ),
        launch_arguments={
            "world": world_file,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_path,
        }.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("soonduck_simulation"),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "soonduck",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    rviz2_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("soonduck_simulation"),
                "config",
                "config.rviz",
            )
        ],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("soonduck_simulation"),
                    "launch",
                    "joystick.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher,
            joint_state_publisher,
            spawn_entity,
            rviz2_launch,
            diff_drive_spawner,
            joint_broad_spawner,
            joystick,
        ]
    )
