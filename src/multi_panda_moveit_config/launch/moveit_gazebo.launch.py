import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch

def generate_launch_description():
    # Declare launch arguments
    arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Name of the Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='X position of the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Y position of the robot'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Z position of the robot'
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='multi_panda',
            description='Name of the robot entity'
        )
    ]

    # Package directories
    robot_moveit_config_path = os.path.join(get_package_share_directory('multi_panda_moveit_config'))

    # --- Gazebo and Spawn ---
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_moveit_config_path,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'entity_name': LaunchConfiguration('entity_name')
        }.items()
    )

    # --- MoveIt ---
    # Build MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("multi_panda", package_name="multi_panda_moveit_config")
        .robot_description(file_path=os.path.join(robot_moveit_config_path, "config", "multi_panda.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(robot_moveit_config_path, "config", "multi_panda.srdf"))
        .trajectory_execution(file_path=os.path.join(robot_moveit_config_path, "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=os.path.join(robot_moveit_config_path, "config", "joint_limits.yaml"))
        .to_moveit_configs()
    )

    # Load MoveIt configuration
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Launch RViz
    rviz = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        arguments + [
            launch_gazebo,
            move_group,
            rviz,
        ]
    )

