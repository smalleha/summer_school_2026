from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensor_interface_node = Node(
        package='sensor_interface',
        executable='sensor_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([sensor_interface_node])
