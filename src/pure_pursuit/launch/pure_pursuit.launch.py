"""Launch the PurePursuit composable node in its own component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            ComposableNodeContainer(
                name="pure_pursuit_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="pure_pursuit",
                        plugin="robonav_training::PurePursuit",
                        name="pure_pursuit",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "lookahead_distance": 0.3,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
