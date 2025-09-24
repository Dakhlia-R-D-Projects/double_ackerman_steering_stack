# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_rgbd_scan_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

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

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('localization')),
            launch_arguments=[
                  ('use_sim_time', use_sim_time),
                  ('params_file', nav2_params_file.perform(context)),
                  ('use_composition', 'False'),
                  ('namespace', namespace_str)
                ]
        ),
    ])

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
        # rviz,
        rtabmap,
        nav2,
    ]

def generate_launch_description():
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
