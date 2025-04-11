import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, SetEnvironmentVariable

def generate_launch_description():
    # Declare arguments
    arguments = LaunchDescription([
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
    ])

    # Get the path to the robot description package
    robot_description_path = os.path.join(get_package_share_directory('multi_panda_description'))
    robot_moveit_config_path = os.path.join(get_package_share_directory('multi_panda_moveit_config'))
    gazebo_controllers_path = os.path.join(get_package_share_directory('multi_panda_description'), 'config', 'gazebo_controller_manager.yaml')
    xacro_file_path = os.path.join(robot_moveit_config_path, 'config', 'multi_panda.urdf.xacro')
    robot_state_description = xacro.process_file(xacro_file_path, mappings={'use_sim' : 'true'}).toprettyxml(indent='  ')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_state_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[os.path.join(robot_description_path, 'worlds')]
    )

    # Launch Gazebo with the specified world
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', [
                '-r -v 1 ',
                PathJoinSubstitution(
                    [FindPackageShare('multi_panda_description'), 'worlds', LaunchConfiguration('world')]
                )
            ])
        ]
    )

    # Spawn the robot model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('entity_name'),
            '-string', robot_state_description,
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Load controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='both',
        parameters=[gazebo_controllers_path],
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_dual_panda_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='dual_panda_arm_controller',
        output='screen',
        arguments=['dual_panda_arm_controller']
    )

    load_left_panda_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='left_panda_hand_controller',
        output='screen',
        arguments=['left_panda_hand_controller']
    )

    load_right_panda_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='right_panda_hand_controller',
        output='screen',
        arguments=['right_panda_hand_controller']
    )

    delay_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    delay_controllers_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_dual_panda_arm_controller,
                      load_left_panda_hand_controller,
                      load_right_panda_hand_controller]
        )
    )

    delay_controller_after_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[controller_manager]
        )
    )

    return LaunchDescription([
        arguments,
        robot_state_publisher,
        gazebo_resource_path,
        launch_gazebo,
        spawn_robot,
        bridge,
        delay_controller_after_publisher,
        delay_joint_state_after_spawn,
        delay_controllers_after_joint_state
    ])