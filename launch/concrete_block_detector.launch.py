from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Start standalone point-cloud concrete-block discovery."""
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("concrete_block_detector"), "config", "detector.yaml"]
        ),
    )
    points_topic = DeclareLaunchArgument("points_topic", default_value="/seyond/points")
    transport = DeclareLaunchArgument("transport", default_value="raw")
    world_model_enabled = DeclareLaunchArgument(
        "world_model_enabled",
        default_value="false",
        description="Write detected blocks to world_model_node (the wall-assembly launch enables this).",
    )
    return LaunchDescription(
        [
            params_file,
            points_topic,
            transport,
            world_model_enabled,
            Node(
                package="concrete_block_detector",
                executable="concrete_block_detector_node",
                name="concrete_block_detector",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "world_model.enabled": LaunchConfiguration("world_model_enabled"),
                        "point_cloud_transport": LaunchConfiguration("transport"),
                    },
                ],
                remappings=[("points", LaunchConfiguration("points_topic"))],
                output="screen",
            ),
        ]
    )
