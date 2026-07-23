"""Point-cloud perception replacement for the concrete-block wall pipeline.

This launches only the pieces consumed by wall assembly: the point-cloud
detector and the persistent world model. Cloudini decoding happens in-process
through the detector's point_cloud_transport subscription. The legacy image
segmentation, mask-cutout, detection-tracking and registration nodes are not
started.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_params = DeclareLaunchArgument(
        "detector_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("concrete_block_detector"), "config", "detector.yaml"]
        ),
    )
    world_model_params = DeclareLaunchArgument(
        "world_model_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("concrete_block_world_model"), "config", "world_model.yaml"]
        ),
    )
    world_model_overlay = DeclareLaunchArgument(
        "world_model_overlay_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("concrete_block_world_model"), "config", "world_model_seed_none.yaml"]
        ),
    )
    points_topic = DeclareLaunchArgument("points_topic", default_value="/seyond/points")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    start_world_model = DeclareLaunchArgument("start_world_model", default_value="true")
    return LaunchDescription(
        [
            detector_params,
            world_model_params,
            world_model_overlay,
            points_topic,
            use_sim_time,
            start_world_model,
            Node(
                package="concrete_block_world_model",
                executable="world_model_node",
                name="world_model_node",
                parameters=[
                    LaunchConfiguration("world_model_params_file"),
                    LaunchConfiguration("world_model_overlay_params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("block_world_model", "/cbp/block_world_model"),
                    ("block_world_model_markers", "/cbp/block_world_model_markers"),
                    ("block_goal_markers", "/cbp/block_goal_markers"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_world_model")),
            ),
            Node(
                package="concrete_block_detector",
                executable="concrete_block_detector_node",
                name="concrete_block_detector",
                parameters=[
                    LaunchConfiguration("detector_params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "world_model.enabled": True,
                        "point_cloud_transport": "cloudini",
                    },
                ],
                remappings=[("points", LaunchConfiguration("points_topic"))],
                output="screen",
            ),
        ]
    )
