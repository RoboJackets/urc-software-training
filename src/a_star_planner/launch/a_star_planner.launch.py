"""Launch the A* planner as a composable node in its own container."""

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
                name="a_star_planner_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="a_star_planner",
                        plugin="robonav_training::AStarPlanner",
                        name="a_star_planner",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "inflation_radius": 0.55,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
