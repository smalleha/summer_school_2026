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

    move_target_node = Node(
        package="learnning_piper",
        executable="move_target",
        name="move_target",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "dx": ParameterValue(dx, value_type=float),
                "dy": ParameterValue(dy, value_type=float),
                "dz": ParameterValue(dz, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dx", default_value="0.2"),
            DeclareLaunchArgument("dy", default_value="0.2"),
            DeclareLaunchArgument("dz", default_value="0.1"),
            move_target_node,
        ]
    )
