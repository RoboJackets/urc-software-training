"""Launch the MapServer composable node in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("map_server"), "maps", "training_map.yaml"]
                ),
            ),
            ComposableNodeContainer(
                name="map_server_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="map_server",
                        plugin="robonav_training::MapServer",
                        name="map_server",
                        parameters=[{"use_sim_time": use_sim_time, "map_yaml_path": map_yaml}],
                    ),
                ],
            ),
        ]
    )
