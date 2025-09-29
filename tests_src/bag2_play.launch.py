# play 3 ros2 bags at the same time

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Velodyne bag player
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'anymal_velodne_undist_ros2', '--clock', '100'],
            output='screen'
        ),
        
        # Livox bag player
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'anymal_livox_undist_ros2', '--clock', '100'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'anymal_hesai_undist_ros2', '--clock', '100'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'anymal_dlio_ros2', '--clock', '100'],
            output='screen'
        ),
        
        # Transform between the LiDAR frames
        # Modify the translation/rotation values based on your actual setup
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'velodyne_lidar'],
            output='screen'
        ),
        
        # Add another transform for the Livox LiDAR
        # Replace 'livox_frame' with the actual frame_id from your Livox data
        # Adjust x y z roll pitch yaw values based on relative positioning
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '3.14159', '0', 'map', 'livox_lidar'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '4.71239', '0', '0', 'map', 'hesai_lidar'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'map', 'map'],
            output='screen'
        ),
        
        # Start RViz2
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )
    ])