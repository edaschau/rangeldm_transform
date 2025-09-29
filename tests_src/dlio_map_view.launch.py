# dlio_map_viewer.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Play the map bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/eda/Desktop/KIT_robotics/RangeLDM/dlio_map'],
            output='screen'
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/jazzy/share/rviz2/default.rviz'],
            output='screen'
        )
    ])