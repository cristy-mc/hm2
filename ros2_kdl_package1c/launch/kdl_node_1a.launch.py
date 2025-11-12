from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    cmd_interface = DeclareLaunchArgument(
        name='cmd_interface',
        default_value='position',  
        description='Command interface of the robot controllers'
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("ros2_kdl_package"),
            "config",
            "kdl_param.yaml",
        ]),
        description="YAML con i parametri del nodo"
    )

    ros2_kdl_node_1a = Node(
        package="ros2_kdl_package",
        executable="ros2_kdl_node_1a",
        name="ros2_kdl_node_1a",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"cmd_interface": LaunchConfiguration("cmd_interface")},
        ],
    )

    return LaunchDescription([
        cmd_interface,
        params_file_arg,
        ros2_kdl_node_1a,
    ])
