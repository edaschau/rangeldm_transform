#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf_transformations
from collections import deque
import threading
import time
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

class LidarAlignmentNode(Node):
    def __init__(self):
        super().__init__('lidar_alignment_node')
        
        # Publishers for aligned point clouds
        self.aligned_velodyne_pub = self.create_publisher(
            PointCloud2, '/aligned/velodyne/points', 10)
        self.aligned_hesai_pub = self.create_publisher(
            PointCloud2, '/aligned/hesai/points', 10)
        self.aligned_livox_pub = self.create_publisher(
            PointCloud2, '/aligned/livox/points', 10)
        self.merged_cloud_pub = self.create_publisher(
            PointCloud2, '/aligned/merged_cloud', 10)
        
        # Publisher for TF (for visualization in RViz2)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 10)
        
        # Store TF tree
        self.tf_buffer = {}
        self.static_tf_buffer = {}
        
        # Store odometry
        self.current_odometry = None
        
        # Thread-safe queue for point clouds
        self.pointcloud_queue = deque(maxlen=100)
        
        # Bag file path
        self.bag_path = '/home/eda/Desktop/KIT_robotics/RangeLDM/merged_bag'
        
        self.get_logger().info('LiDAR Alignment Node initialized')
        
    def parse_tf_message(self, tf_msg, is_static=False):
        """Parse TF message and store transforms"""
        for transform in tf_msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            
            if is_static:
                self.static_tf_buffer[key] = transform
            else:
                # Store with timestamp for dynamic transforms
                if key not in self.tf_buffer:
                    self.tf_buffer[key] = deque(maxlen=1000)
                self.tf_buffer[key].append({
                    'timestamp': transform.header.stamp,
                    'transform': transform
                })
    
    def get_transform_at_time(self, parent_frame, child_frame, timestamp=None):
        """Get transform between frames at specific timestamp"""
        key = (parent_frame, child_frame)
        
        # First check static transforms
        if key in self.static_tf_buffer:
            return self.static_tf_buffer[key]
        
        # Then check dynamic transforms
        if key in self.tf_buffer and len(self.tf_buffer[key]) > 0:
            if timestamp is None:
                # Return latest transform
                return self.tf_buffer[key][-1]['transform']
            else:
                # Find closest transform to requested timestamp
                # For simplicity, return the latest one before the timestamp
                for tf_data in reversed(self.tf_buffer[key]):
                    if tf_data['timestamp'].sec <= timestamp.sec and \
                       tf_data['timestamp'].nanosec <= timestamp.nanosec:
                        return tf_data['transform']
                # If no earlier transform found, return the earliest one
                return self.tf_buffer[key][0]['transform']
        
        return None
    
    def get_transform_chain(self, target_frame, source_frame, timestamp=None):
        """Get complete transform chain from source to target frame"""
        # For ANYmal, we need to handle these chains:
        # velodyne_lidar -> base (direct)
        # hesai_lidar -> box_base -> base
        # livox_lidar -> box_base -> base
        
        if source_frame == 'velodyne_lidar' and target_frame == 'base':
            return self.get_transform_at_time('base', 'velodyne_lidar', timestamp)
        
        elif source_frame == 'hesai_lidar' and target_frame == 'base':
            # Get hesai_lidar -> box_base
            tf1 = self.get_transform_at_time('box_base', 'hesai_lidar', timestamp)
            # Get box_base -> base
            tf2 = self.get_transform_at_time('base', 'box_base', timestamp)
            if tf1 and tf2:
                return self.combine_transforms(tf2, tf1)
            
        elif source_frame == 'livox_lidar' and target_frame == 'base':
            # Get livox_lidar -> box_base
            tf1 = self.get_transform_at_time('box_base', 'livox_lidar', timestamp)
            # Get box_base -> base
            tf2 = self.get_transform_at_time('base', 'box_base', timestamp)
            if tf1 and tf2:
                return self.combine_transforms(tf2, tf1)
        
        return None
    
    def combine_transforms(self, tf1, tf2):
        """Combine two transforms: result = tf1 * tf2"""
        # Extract translations and rotations
        t1 = np.array([tf1.transform.translation.x,
                       tf1.transform.translation.y,
                       tf1.transform.translation.z])
        q1 = [tf1.transform.rotation.x, tf1.transform.rotation.y,
              tf1.transform.rotation.z, tf1.transform.rotation.w]
        
        t2 = np.array([tf2.transform.translation.x,
                       tf2.transform.translation.y,
                       tf2.transform.translation.z])
        q2 = [tf2.transform.rotation.x, tf2.transform.rotation.y,
              tf2.transform.rotation.z, tf2.transform.rotation.w]
        
        # Convert to matrices
        M1 = tf_transformations.quaternion_matrix(q1)
        M1[:3, 3] = t1
        M2 = tf_transformations.quaternion_matrix(q2)
        M2[:3, 3] = t2
        
        # Combine
        M_combined = M1 @ M2
        
        # Extract result
        t_combined = M_combined[:3, 3]
        q_combined = tf_transformations.quaternion_from_matrix(M_combined)
        
        # Create combined transform
        result = TransformStamped()
        result.header = tf1.header
        result.child_frame_id = tf2.child_frame_id
        result.transform.translation.x = t_combined[0]
        result.transform.translation.y = t_combined[1]
        result.transform.translation.z = t_combined[2]
        result.transform.rotation.x = q_combined[0]
        result.transform.rotation.y = q_combined[1]
        result.transform.rotation.z = q_combined[2]
        result.transform.rotation.w = q_combined[3]
        
        return result
    
    def transform_pointcloud(self, points, transform):
        """Apply transform to point cloud"""
        if transform is None:
            self.get_logger().warn('Transform is None, returning original points')
            return points
        
        # Extract rotation and translation
        q = [transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w]
        t = np.array([transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z])
        
        # Convert quaternion to rotation matrix
        R = tf_transformations.quaternion_matrix(q)[:3, :3]
        
        # Apply transformation: p_transformed = R * p + t
        transformed_points = (R @ points.T).T + t
        
        return transformed_points
    
    def process_pointcloud(self, cloud_msg, source_frame):
        """Process and align a point cloud"""
        # Extract points from PointCloud2
        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if len(points) == 0:
            return None
        
        points = np.array(points)
        
        # Get transform from source frame to base frame
        transform = self.get_transform_chain('base', source_frame, cloud_msg.header.stamp)
        
        if transform is None:
            self.get_logger().warn(f'Could not find transform from {source_frame} to base')
            return None
        
        # Transform points to base frame
        transformed_points = self.transform_pointcloud(points, transform)
        
        # Create aligned point cloud message
        header = cloud_msg.header
        header.frame_id = 'base'
        aligned_cloud = pc2.create_cloud_xyz32(header, transformed_points)
        
        return aligned_cloud
    
    def merge_pointclouds(self, clouds):
        """Merge multiple point clouds into one"""
        if not clouds:
            return None
        
        all_points = []
        timestamp = None
        
        for cloud in clouds:
            if cloud is not None:
                points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
                all_points.extend(points)
                if timestamp is None:
                    timestamp = cloud.header.stamp
        
        if not all_points:
            return None
        
        # Create merged cloud
        header = PointCloud2().header
        header.stamp = timestamp
        header.frame_id = 'base'
        
        merged_cloud = pc2.create_cloud_xyz32(header, all_points)
        return merged_cloud
    
    def process_bag(self):
        """Process the rosbag and align point clouds"""
        self.get_logger().info(f'Processing bag: {self.bag_path}')
        
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_type.name: topic_type.type for topic_type in topic_types}
        
        # Process messages
        frame_count = 0
        current_frame_clouds = {}
        
        while reader.has_next():
            (topic, data, timestamp_ns) = reader.read_next()
            
            # Create ROS timestamp
            stamp = rclpy.time.Time(nanoseconds=timestamp_ns).to_msg()
            
            # Process TF messages
            if topic == '/tf_static':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=True)
                self.tf_static_pub.publish(msg)  # Republish for RViz
                
            elif topic == '/tf':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=False)
                self.tf_pub.publish(msg)  # Republish for RViz
                
            # Process odometry
            elif topic == '/boxi/dlio/lidar_map_odometry':
                msg_type = get_message(type_map[topic])
                self.current_odometry = deserialize_message(data, msg_type)
                
            # Process point clouds
            elif topic in ['/anymal/velodyne/points_undistorted',
                          '/boxi/hesai/points_undistorted',
                          '/boxi/livox/points_undistorted']:
                
                msg_type = get_message(type_map[topic])
                cloud_msg = deserialize_message(data, msg_type)
                
                # Determine source frame
                if 'velodyne' in topic:
                    source_frame = 'velodyne_lidar'
                    publisher = self.aligned_velodyne_pub
                elif 'hesai' in topic:
                    source_frame = 'hesai_lidar'
                    publisher = self.aligned_hesai_pub
                elif 'livox' in topic:
                    source_frame = 'livox_lidar'
                    publisher = self.aligned_livox_pub
                
                # Process and align point cloud
                aligned_cloud = self.process_pointcloud(cloud_msg, source_frame)
                
                if aligned_cloud:
                    # Publish aligned individual cloud
                    publisher.publish(aligned_cloud)
                    
                    # Store for merging
                    current_frame_clouds[source_frame] = aligned_cloud
                    
                    # If we have all three clouds, merge and publish
                    if len(current_frame_clouds) == 3:
                        merged = self.merge_pointclouds(list(current_frame_clouds.values()))
                        if merged:
                            self.merged_cloud_pub.publish(merged)
                        current_frame_clouds = {}
                        
                        frame_count += 1
                        if frame_count % 10 == 0:
                            self.get_logger().info(f'Processed {frame_count} frames')
                
                # Small delay to allow visualization
                time.sleep(0.01)
        
        self.get_logger().info(f'Finished processing. Total frames: {frame_count}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarAlignmentNode()
    
    # Create a thread for bag processing
    bag_thread = threading.Thread(target=node.process_bag)
    bag_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        bag_thread.join()

if __name__ == '__main__':
    main()