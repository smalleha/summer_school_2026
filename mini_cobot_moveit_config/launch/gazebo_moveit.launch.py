from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "mini_cobot_moveit_config"
    package_share = Path(get_package_share_directory(package_name))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_z = LaunchConfiguration("start_z")
    start_yaw = LaunchConfiguration("start_yaw")

    resource_paths = sorted(
        {
            str(Path(get_package_share_directory("scout_description")).parent),
            str(Path(get_package_share_directory("agx_arm_description")).parent),
            str(Path(get_package_share_directory("realsense2_description")).parent),
        }
    )

    moveit_config = (
        MoveItConfigsBuilder("mini_cobot", package_name=package_name)
        .robot_description(file_path="config/mini_cobot.gazebo.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "gui": gui,
            "pause": "true",
            "verbose": verbose,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "mini_cobot",
            "-topic",
            "robot_description",
            "-x",
            start_x,
            "-y",
            start_y,
            "-z",
            start_z,
            "-Y",
            start_yaw,
            "-timeout",
            "60",
        ],
    )

    joint_state_broadcaster_spawner = Node(
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
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    unpause_gazebo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )
    unpause_gazebo_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[unpause_gazebo],
        )
    )
    start_joint_state_broadcaster_after_unpause = RegisterEventHandler(
        OnProcessExit(
            target_action=unpause_gazebo,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    start_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    start_gripper_controller_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", str(package_share / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt plugin.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo client.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose Gazebo server logging.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(package_share / "worlds" / "test.world"),
                description="Absolute path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "start_x",
                default_value="0.0",
                description="Robot spawn x position.",
            ),
            DeclareLaunchArgument(
                "start_y",
                default_value="0.0",
                description="Robot spawn y position.",
            ),
            DeclareLaunchArgument(
                "start_z",
                default_value="0.0",
                description="Robot spawn z position.",
            ),
            DeclareLaunchArgument(
                "start_yaw",
                default_value="0.0",
                description="Robot spawn yaw.",
            ),
            SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
            AppendEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
            ),
            *[
                AppendEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=path)
                for path in resource_paths
            ],
            *[
                AppendEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=path)
                for path in resource_paths
            ],
            gazebo,
            robot_state_publisher,
            spawn_robot,
            unpause_gazebo_after_spawn,
            start_joint_state_broadcaster_after_unpause,
            start_arm_controller_after_joint_state_broadcaster,
            start_gripper_controller_after_arm_controller,
            move_group,
            rviz,
        ]
    )
