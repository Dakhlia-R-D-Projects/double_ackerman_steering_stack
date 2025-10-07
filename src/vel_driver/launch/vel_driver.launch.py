from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    vel_driver_param_path_arg = DeclareLaunchArgument(
        'vel_driver_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('vel_driver'),
            'config',
            'vel_driver.yaml'
        ]),
        description='Path to the vel_driver parameter file'
    )

    # Create the node
    vel_driver_node = Node(
        package='vel_driver',
        executable='vel_driver_node',
        name='vel_driver',
        output='screen',
        parameters=[LaunchConfiguration('vel_driver_param_path')]
    )

    return LaunchDescription([
        vel_driver_param_path_arg,
        vel_driver_node
    ])
