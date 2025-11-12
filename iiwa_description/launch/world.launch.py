import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Argomento per mostrare o meno la GUI di Gazebo
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Show Gazebo GUI'
    )

    # Trova il pacchetto iiwa_description
    pkg_share = FindPackageShare('iiwa_description')

    # Path assoluto della world file
    world_path = PathJoinSubstitution([
        pkg_share,
        'gazebo',
        'worlds',
        'empty.world'
    ])

    # Include del launcher ufficiale di Gazebo (ros_gz_sim)
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

    return LaunchDescription([
        gui_arg,
        gz_sim_launch
    ])
