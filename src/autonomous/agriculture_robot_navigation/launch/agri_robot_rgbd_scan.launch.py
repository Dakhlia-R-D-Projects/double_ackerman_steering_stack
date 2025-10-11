# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_rgbd_scan.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    remappings_right_camera=[
      ('/rgb/image',       'right_camera/rgb/image'),
      ('/depth/image',     'right_camera/depth/image'),
      ('/rgb/camera_info', 'right_camera/rgb/camera_info'),
      ('rgbd_image', 'right_camera/rgbd_image')]
    remappings_left_camera=[
      ('/rgb/image',       'left_camera/rgb/image'),
      ('/depth/image',     'left_camera/depth/image'),
      ('/rgb/camera_info', 'left_camera/rgb/camera_info'),
      ('rgbd_image', 'left_camera/rgbd_image')]
    remappings_slam=[
        ('rgbd_image0', 'left_camera/rgbd_image'),
        ('rgbd_image1', 'right_camera/rgbd_image'),
        ('odom', 'odom'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('agriculture_robot_navigation'),
                'config',
                'rtabmap.yaml'
            ]),
            description='Path to the rtabmapconfiguration file'
        ),
            # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync',name="left_rgbd", output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings_left_camera),
        Node(
            package='rtabmap_sync', executable='rgbd_sync',name="right_rgbd", output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings_right_camera),
        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=remappings_slam,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[LaunchConfiguration('config_file'),
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True',
               'initial_pose':'5.0 -4.6 0 0 0 -1.57'}], # set initial pose x y z roll pitch yaw
            remappings=remappings_slam),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[parameters]),

        Node(
            package='rtabmap_util', executable='obstacles_detection',name="right_obstacles_detection", output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('cloud', 'right_camera/points'),
                        ('obstacles', 'right_camera/points/obstacles'),
                        ('ground', 'right_camera/points/ground')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection',name="left_obstacles_detection", output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('cloud', 'left_camera/points'),
                        ('obstacles', 'left_camera/points/obstacles'),
                        ('ground', 'left_camera/points/ground')]),
])
