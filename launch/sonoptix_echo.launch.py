from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    config = PathJoinSubstitution(
        [FindPackageShare("sonoptix_echo_driver_ros"), "config", "sonoptix_echo.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            DeclareLaunchArgument(
                "namespace",
                default_value=EnvironmentVariable("ROBOT_NAME", default_value="bluebot"),
            ),
            Node(
                package="sonoptix_echo_driver_ros",
                executable="sonoptix_echo_driver.py",
                name="sonoptix_echo_driver_node",
                namespace=namespace,
                output="screen",
                parameters=[config],
            ),
        ]
    )
