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
    scene_discovery_params = DeclareLaunchArgument(
        "scene_discovery_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("concrete_block_detector"),
                "config",
                "scene_discovery_defaults.yaml",
            ]
        ),
    )
    points_topic = DeclareLaunchArgument("points_topic", default_value="/seyond/points")
    transport = DeclareLaunchArgument("transport", default_value="raw")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    return LaunchDescription(
        [
            params_file,
            scene_discovery_params,
            points_topic,
            transport,
            use_sim_time,
            Node(
                package="concrete_block_detector",
                executable="concrete_block_detector_node",
                name="concrete_block_detector",
                parameters=[
                    LaunchConfiguration("params_file"),
                    LaunchConfiguration("scene_discovery_params"),
                    {
                        "point_cloud_transport": LaunchConfiguration("transport"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=[
                    ("points", LaunchConfiguration("points_topic")),
                    # point_cloud_transport resolves plugin topics separately from
                    # the base topic, so remap Cloudini explicitly as well.
                    (
                        "points/cloudini",
                        [LaunchConfiguration("points_topic"), "/cloudini"],
                    ),
                ],
                output="screen",
            ),
        ]
    )
