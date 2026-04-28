from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    print(f"package_path: {absolute_file_path}")

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    description_package = "mh2_description"
    description_filepath = path.join("urdf", "robot.urdf.xacro")
    rviz_config_package = "mh2_bringup"
    rviz_config_filepath = path.join("config", "moveit.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(description_package),
        description_filepath
    ])
    robot_description = {
    'robot_description': ParameterValue(
        Command(['xacro ', xacro_file_path]),
        value_type=str
    )
}
    
    rviz_config_path = path.join(
        get_package_share_directory(rviz_config_package),
        rviz_config_filepath
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     output="log",
        #     arguments=[
        #         "-d", rviz_config_path,  
        #         "--ros-args",
        #         "--log-level",
        #         log_level,
        #     ],
        #     parameters=[{"use_sim_time": use_sim_time}],
        # ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="log",
            parameters=[
                robot_description,
                path.join(get_package_share_directory("mh2_bringup"), "config", "ros2_controllers.yaml"),
                {"use_sim_time": use_sim_time}
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_controllers",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "arm_left_controller",
                "arm_right_controller",
                "--controller-manager-timeout", "10",
            ],
            parameters=[{"use_sim_time": use_sim_time}], 
        ),
    ]

    move_group_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("mh2_moveit_config"),
            "launch",
            "move_group.launch.py"
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level
        }.items()
    )

    return LaunchDescription(declared_arguments + nodes + [move_group_launch])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        # DeclareLaunchArgument(
        #     "rviz_config",
        #     default_value=path.join(
        #         get_package_share_directory("mh2_bringup"),
        #         "config",
        #         "moveit.rviz",
        #     ),
        #     description="Path to configuration for RViz2.",
        # ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log level.",
        ),
    ]
