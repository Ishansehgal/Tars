from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    pkg_tars = get_package_share_directory('tars_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Environment Variable to ensure plugins are found
    env_var = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )


    # Process URDF
    xacro_file = os.path.join(pkg_tars, 'urdf', 'tars_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Launch Gazebo Sim with empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'tars_robot'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/model/tars_robot/pose@geometry_msgs/msg/PoseWithCovarianceStamped[ignition.msgs.Pose_V',
        ],
        output='screen'
    )

    # Broadcaster
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Controller
    jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        env_var,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb,
                on_exit=[jtc],
            )
        )
    ])
