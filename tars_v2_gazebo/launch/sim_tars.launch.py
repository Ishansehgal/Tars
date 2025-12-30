from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, ExecuteProcess)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get package directories
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_tars_robot_description = get_package_share_directory('tars_v2_gazebo')

    # Set paths for Ignition Gazebo launch
    ign_gz_launch_path = PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    
    # Path to the SDF file for Gazebo
    sdf_file = os.path.join(pkg_tars_robot_description, 'urdf', 'tars.sdf')
    
    # Path to the directory containing model files
    model_path = os.path.join(pkg_tars_robot_description, 'urdf')

    # Read the SDF file content
    with open(sdf_file, 'r') as file:
        sdf_content = file.read()

    return LaunchDescription([
        # Set Ignition resource path to include our models directory
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_path),
        
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ign_gz_launch_path),
            launch_arguments={
                'ign_args': '-r empty.sdf --headless-rendering',
            }.items(),
        ),

        # Spawn the robot in Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'service', '-s', '/world/empty/create',
                    '--reqtype', 'ignition.msgs.EntityFactory', 
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'sdf_filename: "{sdf_file}", name: "tars"'],
            output='screen'
        ),

        # Bridge to convert joint states from Ignition to ROS
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='ros_ign_bridge_joint_states',
            arguments=[
                '/world/empty/model/tars/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
            ],
            output='screen'
        ),
        
        # Bridge to convert TF data from Ignition to ROS
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='ros_ign_bridge_tf',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
            ],
            output='screen'
        ),
    ])