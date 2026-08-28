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
                name="particle_filter_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="particle_filter",
                        plugin="particle_filter::ParticleFilter",
                        name="particle_filter",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "odom_topic": "/odometry/filtered",
                                "scan_topic": "/lidar/scan",
                                "map_topic": "/map",
                                "particle_cloud_topic": "/particle_cloud",
                                "estimated_pose_topic": "/amcl_pose",
                                "base_frame": "base_footprint",
                                "lidar_frame": "lidar_link",
                                "random_particle_percent": 5.0,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
