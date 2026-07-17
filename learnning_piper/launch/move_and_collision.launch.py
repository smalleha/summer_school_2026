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
    box_size_x = LaunchConfiguration("box_size_x")
    box_size_y = LaunchConfiguration("box_size_y")
    box_size_z = LaunchConfiguration("box_size_z")

    move_and_collision_node = Node(
        package="learnning_piper",
        executable="move_and_collision",
        name="move_and_collision",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "dx": ParameterValue(dx, value_type=float),
                "dy": ParameterValue(dy, value_type=float),
                "dz": ParameterValue(dz, value_type=float),
                "box_size_x": ParameterValue(box_size_x, value_type=float),
                "box_size_y": ParameterValue(box_size_y, value_type=float),
                "box_size_z": ParameterValue(box_size_z, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dx", default_value="0.20"),
            DeclareLaunchArgument("dy", default_value="0.20"),
            DeclareLaunchArgument("dz", default_value="0.10"),
            DeclareLaunchArgument("box_size_x", default_value="0.18"),
            DeclareLaunchArgument("box_size_y", default_value="0.08"),
            DeclareLaunchArgument("box_size_z", default_value="0.20"),
            move_and_collision_node,
        ]
    )
