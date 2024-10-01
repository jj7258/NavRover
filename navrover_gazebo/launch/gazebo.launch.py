from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('navrover_description'),
                 'urdf', 'navrover.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('navrover_control'),
            'config',
            'navrover_controllers.yaml',
        ]
    )

    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'navrover', '-allow_renaming', 'true'],
    )

    # ros2_control related nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file', robot_controllers],
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'effort_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    # Bridge to connect ROS 2 and Gazebo
    bridge_params = PathJoinSubstitution(
        [FindPackageShare('navrover_gazebo'), 'config', 'bridge_parameters.yaml']
    )
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', bridge_params],  # Ensure the path is correctly formed
        ],
        output='screen'
    )

    # stability_controller_node = Node(
    #     package='navrover_control',
    #     executable='stability_controller',
    #     name='stability_controller',
    #     output='screen'
    # )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 3 empty.sdf'])]),
        # RegisterEventHandler for OnProcessExit with on_exit argument
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[effort_controller_spawner],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        start_gazebo_ros_bridge_cmd,
        # stability_controller_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])