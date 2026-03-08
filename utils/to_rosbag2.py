# rosbag1 to rosbag2 conversion script

from pathlib import Path
from rosbags.convert import converter

def main():
    # Input ROS1 bag file path
    ros1_bag = Path('GrandTour/2024-11-02-17-10-25_tf_model.bag')
    
    # Output ROS2 bag directory path
    ros2_bag = Path('anymal_tf_ros2')

    print(f"Converting ROS1 bag '{ros1_bag}' to ROS2 bag '{ros2_bag}'...")
    
    # Perform the conversion
    # By default, topic names will be kept the same
    # If you need to rename topics, you can provide a topics dict
    # e.g., topics={'/old_topic_name': '/new_topic_name'}
    converter.convert(ros1_bag, ros2_bag)
    
    print(f"Conversion complete! ROS2 bag created at: {ros2_bag}")
    print("\nYou can now use the following commands to work with this bag:")
    print("1. Play the bag:")
    print(f"   ros2 bag play {ros2_bag}")
    print("\n2. View with RViz2:")
    print("   ros2 run rviz2 rviz2")
    print("   (Then in RViz2, add a PointCloud2 display and set the topic to /anymal/velodyne/points)")

if __name__ == "__main__":
    main()