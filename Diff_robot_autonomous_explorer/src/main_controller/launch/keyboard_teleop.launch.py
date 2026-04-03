import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    # Keyboard teleop node
    keyboard_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="keyboard_teleop",
        prefix="xterm -e",  # opens in a new terminal for keyboard input
        remappings=[("/cmd_vel", "/main_controller/cmd_vel")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            keyboard_teleop
        ]
    )
