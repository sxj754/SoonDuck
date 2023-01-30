import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdf_file_name = "soonduck.urdf"
    urdf = os.path.join(
        get_package_share_directory("soonduck_simulation"), "urdf", urdf_file_name
    )
    with open(urdf, "r") as file:
        robot_desc = file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        arguments=[urdf],
    )

    joint_state_publisher = Node(
        package="soonduck_simulation",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "soonduck", "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher,
            joint_state_publisher,
            spawn_entity,
        ]
    )


# def generate_launch_description():
#     # Launches Gazebo
#     gazeboLaunch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [
#                 os.path.join(
#                     get_package_share_directory("gazebo_ros"),
#                     "launch",
#                     "gazebo.launch.py",
#                 )
#             ]
#         ),
#         # launch_arguments={"world": world_path}.items(),
#     )

#     filenameURDF = "soonduck.urdf"
#     # filenameYaml = "rov_description_params.yaml"

#     # Path to Xacro file of robot
#     path_to_urdf = os.path.join(
#         get_package_share_directory("soonduck_simulation"), "urdf", filenameURDF
#     )
#     # with open(path_to_urdf, 'r') as infp:
#     # //  robot_desc = infp.read()

#     robot_desc: ParameterValue = ParameterValue(
#         Command(["xacro ", path_to_urdf]),
#         value_type=str
#         # Command("xacro rov.xacro params_path:=rov_description_params.yaml")
#     )

#     params = {"robot_description": robot_desc}

#     # Publishes the state of the robot
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[params],
#         arguments=[path_to_urdf],
#     )

#     # joint_state_publisher_node = Node(
#     #     package="joint_state_publisher",
#     #     executable="joint_state_publisher",
#     #     arguments=[path_to_urdf],
#     # )

#     # spawn_entity_node = Node(
#     #     package="gazebo_ros",
#     #     executable="spawn_entity.py",
#     #     arguments=[
#     #         "-entity",
#     #         "rov",
#     #         "-topic",
#     #         "/robot_description",
#     #         "-x",
#     #         "0",
#     #         "-y",
#     #         "2",
#     #         "-z",
#     #         "0",
#     #         # 'output', 'screen',
#     #     ],
#     # )

#     # rviz = Node(
#     #     package="rviz2",
#     #     executable="rviz2",
#     #     arguments=['-d' + os.pazebo'), 'config', 'rov.rviz')],

#     # )

#     return LaunchDescription(
#         [
#             gazeboLaunch,
#             robot_state_publisher_node,
#             # joint_state_publisher_node,
#             # spawn_entity_node,
#             # rviz,
#         ]
#     )
