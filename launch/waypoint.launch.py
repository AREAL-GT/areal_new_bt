from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='areal_landing_px4_communication',
            executable='px4_pos_set_move_act',

        ),
        Node(
            package='areal_landing_px4_communication',
            executable='px4_control_command_srv',
        ),
    ])