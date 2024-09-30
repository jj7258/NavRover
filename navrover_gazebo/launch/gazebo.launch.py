from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
        

    
    robotXacroName = 'navrover' # Name of the xacro file
    # share_dir = get_package_share_directory('navrover_description')


    # xacro_file = os.path.join(share_dir, 'urdf', 'navrover.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    # robot_urdf = robot_description_config.toxml()
    
    namePackage = 'navrover_description'
    
    modelFileRelativePath = 'urdf/navrover.xacro'
    
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    
    
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    # def remove_comments(text):
    #     pattern = r'<!--(.*?)-->'
    #     return re.sub(pattern, '', text, flags=re.DOTALL)

    # # Remove comments from the robot description
    # robotDescription = remove_comments()



    # Gazebo Harmonic launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                          'launch', 'gz_sim.launch.py'))

    # gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', '-v', '4'],
    #     output='screen'
    # )
    
    # Spawning the the empty world 
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 
                                                                                       'on_exit_shutdown': 'true'}.items())

    # Spawn the robot in Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robotDescription,
             'use_sim_time': True}])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # Bridge to connect ROS 2 and Gazebo
    bridge_params = os.path.join(
        get_package_share_directory('navrover_gazebo'), 
        'config', 
        'bridge_parameters.yaml')
    
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    #     output='screen'
    # )
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robotDescription, os.path.join(get_package_share_directory('navrover_control'), 'config', 'navrover_controllers.yaml')],
        output='screen',
        remappings=[("~/robot_description", "/robot_description")],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazeboLaunch,
        spawn_entity,
        start_gazebo_ros_bridge_cmd,
        controller_manager,  # Add the controller manager node
    ])