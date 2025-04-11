from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_description_pkg = get_package_share_directory("multi_panda_description")
    moveit_config_pkg = get_package_share_directory("multi_panda_moveit_config")
    demo_pkg = get_package_share_directory("multi_panda_demos")

    return LaunchDescription([
        # Set the use_sim_time parameter
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="multi_panda_demos",
            executable="pick_place",
            name="pick_place_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}]
        )
    ])
