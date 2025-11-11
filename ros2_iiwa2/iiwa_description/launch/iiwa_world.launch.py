from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition

def generate_launch_description():
   
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', 
        description='Avvia Gazebo con l/interfaccia grafica'
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name', default_value='iiwa', 
        description='Nome del robot in simulazione'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', 
        description='Usa il tempo di simulazione (Gazebo)'
    )


    pkg_share = FindPackageShare('iiwa_description')
    world_path = PathJoinSubstitution([pkg_share, 'gazebo', 'worlds', 'empty.world'])
    xacro_iiwa = PathJoinSubstitution([pkg_share, "gazebo", "iiwa_gazebo.xacro"])

    robot_description_content = Command(['xacro', ' ', xacro_iiwa])
    robot_description_param = {'robot_description': robot_description_content}


    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'gz_args': [TextSubstitution(text='-r '), world_path], 
        }.items(),
    )
 
 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description_param, 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )


    iiwa_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_iiwa',
        output='screen',
        arguments=[
            '-string', robot_description_content, 
            '-entity', LaunchConfiguration('model_name'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '-0.3',    
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-filetype', 'urdf',     
            '-unpause'  
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controllers after URDF is spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_spawner_node,
            on_exit=[joint_state_broadcaster],
        )
    )

    delay_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[velocity_controller],
        )
    )

    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/camera:=/camera_ee',
        ],
        output='screen'
    )


    return LaunchDescription([
        gui_arg, 
        model_name_arg, 
        use_sim_time_arg,
        gz_sim_launch,
        robot_state_publisher_node,
        iiwa_spawner_node,
        delay_joint_state_broadcaster,
        delay_velocity_controller,
        bridge_camera
    ])
