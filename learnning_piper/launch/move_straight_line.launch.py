from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_moveit_config_v5")
        .to_moveit_configs()
    )

    dx = LaunchConfiguration("dx")
    dy = LaunchConfiguration("dy")
    dz = LaunchConfiguration("dz")
    eef_step = LaunchConfiguration("eef_step")
    jump_threshold = LaunchConfiguration("jump_threshold")
    min_fraction = LaunchConfiguration("min_fraction")
    display_time = LaunchConfiguration("display_time")
    execute = LaunchConfiguration("execute")

    move_straight_line_node = Node(
        package="learnning_piper",
        executable="move_straight_line",
        name="move_straight_line",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "dx": ParameterValue(dx, value_type=float),
                "dy": ParameterValue(dy, value_type=float),
                "dz": ParameterValue(dz, value_type=float),
                "eef_step": ParameterValue(eef_step, value_type=float),
                "jump_threshold": ParameterValue(jump_threshold, value_type=float),
                "min_fraction": ParameterValue(min_fraction, value_type=float),
                "display_time": ParameterValue(display_time, value_type=float),
                "execute": ParameterValue(execute, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dx", default_value="0.20"),
            DeclareLaunchArgument("dy", default_value="0.0"),
            DeclareLaunchArgument("dz", default_value="0.0"),
            DeclareLaunchArgument("eef_step", default_value="0.01"),
            DeclareLaunchArgument("jump_threshold", default_value="0.0"),
            DeclareLaunchArgument("min_fraction", default_value="0.95"),
            DeclareLaunchArgument("display_time", default_value="2.0"),
            DeclareLaunchArgument("execute", default_value="true"),
            move_straight_line_node,
        ]
    )
