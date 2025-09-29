from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, SetRemap
import os
from launch.conditions import IfCondition, UnlessCondition

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('agriculture_robot_navigation'),
                              'config',
                              'nav2.yaml'
                              ]),
                          description='Nav2 parameters'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Localization mode'),
]

def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('agriculture_robot_navigation')
    agri_pkg = get_package_share_directory('agriculture_robot_navigation')
    view_pkg = get_package_share_directory('agriculture_robot_viz')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('agriculture_robot_navigation'),'config/nav2.yaml']
    )

    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'nav2.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [view_pkg, 'launch', 'view_navigation.launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [agri_pkg,"launch",'agri_robot_rgbd_scan.launch.py'])

    namespace_str = namespace.perform(context)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', use_sim_time)
        ]
    )
    return [
        # Nodes to launch
        rviz,
        rtabmap,
        # nav2,
    ]

def generate_launch_description():
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
