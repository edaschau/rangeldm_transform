#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition  # <-- Missing import
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    bag_path = LaunchConfiguration('bag_path', 
        default='/home/eda/Desktop/KIT_robotics/RangeLDM/anymal_merged')
    
    config_file = LaunchConfiguration('config_file',
        default=os.path.expanduser('/home/eda/Desktop/KIT_robotics/RangeLDM/mola/grand_tour.yaml'))
    
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('bag_path', default_value=bag_path),
        DeclareLaunchArgument('config_file', default_value=config_file),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),
        
        # Play the rosbag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path,
                 '--rate', '1.0',  # Real-time playback
                 '--topics',
                 '/tf', '/tf_static',
                 '/anymal/velodyne/points_undistorted',
                 '/boxi/hesai/points',
                 '/boxi/livox/points_undistorted'],
            output='screen',
            shell=True
        ),
        
        # Wait for bag to start, then launch MOLA
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['mola-cli',
                        config_file,
                         '--verbosity', 'INFO'],
                    output='screen'
                ),
            ]
        ),
        
        # Optional: Launch RViz for visualization
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.expanduser('/home/eda/Desktop/KIT_robotics/RangeLDM/mola/gt.rviz')],
                    condition=IfCondition(use_rviz)  # <-- Fixed: removed 'launch.conditions'
                )
            ]
        ),
    ])