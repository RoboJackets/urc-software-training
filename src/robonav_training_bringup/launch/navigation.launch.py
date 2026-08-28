"""Starter for the Lesson 12 navigation-stack launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Return a valid launch description before the stack is wired together."""
    # TODO(Lesson 12): declare the map argument, include the EKF launch, and add
    # the completed project components to a ComposableNodeContainer.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulator's /clock instead of wall-clock time.",
            ),
        ]
    )
