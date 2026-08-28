from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    rviz_config = LaunchConfiguration("rviz_config")
    bridge_config = LaunchConfiguration("bridge_config")
    description_share = FindPackageShare("robonav_training_description")
    bringup_share = FindPackageShare("robonav_training_bringup")

    robot_xacro = PathJoinSubstitution(
        [description_share, "urdf", "robonav_training_robot.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [description_share, "config", "ros2_control.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                robot_xacro,
                " use_ros2_control:=true",
                " use_gazebo:=true",
                " ros2_control_config:=",
                controllers_file,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo's /clock instead of wall-clock time.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="robonav_training_robot",
                description="Name assigned to the robot model spawned in Gazebo.",
            ),
            DeclareLaunchArgument(
                "world",
                # The training world matches map_server/maps/training_map.pgm, so localization
                # and planning work out of the box. Pass world:=<...>/empty.sdf for
                # a bare world (e.g. teleop-only testing).
                default_value=PathJoinSubstitution(
                    [bringup_share, "worlds", "training_world.sdf"]
                ),
                description="Absolute path to the SDF world loaded by Gazebo.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "robonav_training.rviz"]
                ),
                description="Absolute path to the RViz display configuration.",
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "gz_bridge.yaml"]
                ),
                description="Absolute path to the Gazebo-to-ROS bridge configuration.",
            ),
            gazebo,
            gz_bridge,
            state_publisher,
            rviz,
            TimerAction(period=2.0, actions=[spawn_robot]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[
                        joint_state_broadcaster,
                        diff_drive_controller,
                    ],
                )
            ),
        ]
    )
