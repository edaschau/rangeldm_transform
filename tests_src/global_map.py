#!/usr/bin/env python3
# NOT WORKING
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
from scipy.spatial import cKDTree
import open3d as o3d

class LidarVisNode(Node):
    def __init__(self):
        super().__init__('lidar_visualization_node')
        
        # Publishers for transformed point clouds (no global accumulation)
        self.velodyne_pub = self.create_publisher(PointCloud2, '/world/velodyne/points', 10)
        self.hesai_pub = self.create_publisher(PointCloud2, '/world/hesai/points', 10)
        self.livox_pub = self.create_publisher(PointCloud2, '/world/livox/points', 10)
        
        # Publisher for TF (for visualization in RViz2)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 10)
        
        # Store TF tree
        self.tf_buffer = {}
        self.static_tf_buffer = {}
        
        # Robot odometry in world frame
        self.robot_pose_world = np.eye(4)  # Identity matrix - start at origin
        
        # Bag file path
        self.bag_path = '/home/eda/Desktop/KIT_robotics/RangeLDM/merged_bag'
        
        # Add flag to track valid odometry
        self.has_valid_odometry = False
        
        self.get_logger().info('LiDAR Visualization Node initialized')
    
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
                for tf_data in reversed(self.tf_buffer[key]):
                    if tf_data['timestamp'].sec <= timestamp.sec and \
                       tf_data['timestamp'].nanosec <= timestamp.nanosec:
                        return tf_data['transform']
                # If no earlier transform found, return the earliest one
                return self.tf_buffer[key][0]['transform']
        
        return None
    
    def get_transform_chain(self, target_frame, source_frame, timestamp=None):
        """Get complete transform chain from source to target frame"""
        if source_frame == 'velodyne_lidar' and target_frame == 'base':
            return self.get_transform_at_time('base', 'velodyne_lidar', timestamp)
        
        elif source_frame == 'hesai_lidar' and target_frame == 'base':
            # Get hesai_lidar -> box_base -> base
            tf1 = self.get_transform_at_time('box_base', 'hesai_lidar', timestamp)
            tf2 = self.get_transform_at_time('base', 'box_base', timestamp)
            if tf1 and tf2:
                return self.combine_transforms(tf2, tf1)
            
        elif source_frame == 'livox_lidar' and target_frame == 'base':
            # Get livox_lidar -> box_base -> base
            tf1 = self.get_transform_at_time('box_base', 'livox_lidar', timestamp)
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
    
    def update_robot_pose(self, odom_msg):
        """Update the robot's pose in world frame based on odometry"""
        # Extract position and orientation from odometry
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        
        # Create transformation matrix from odometry (this is hesai_lidar -> world)
        q = [quat.x, quat.y, quat.z, quat.w]
        R_hesai_to_world = tf_transformations.quaternion_matrix(q)
        R_hesai_to_world[:3, 3] = [pos.x, pos.y, pos.z]
        
        # Get the transform from hesai_lidar to base (if available)
        hesai_to_base = np.eye(4)  # Default identity matrix
        tf_hesai_to_base = self.get_transform_chain('base', 'hesai_lidar')
        if tf_hesai_to_base:
            q_offset = [tf_hesai_to_base.transform.rotation.x,
                    tf_hesai_to_base.transform.rotation.y,
                    tf_hesai_to_base.transform.rotation.z,
                    tf_hesai_to_base.transform.rotation.w]
            t_offset = [tf_hesai_to_base.transform.translation.x,
                    tf_hesai_to_base.transform.translation.y,
                    tf_hesai_to_base.transform.translation.z]
            hesai_to_base = tf_transformations.quaternion_matrix(q_offset)
            hesai_to_base[:3, 3] = t_offset
        
        # Try 180 degree rotation (instead of 90) to correct the flipped orientation
        angle = np.pi  # 180 degrees
        rotation_z_180 = np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle),  np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Calculate the base frame position in world frame
        # Corrected order: First apply hesai->base transform, then apply world->hesai transform
        self.robot_pose_world = rotation_z_180 @ R_hesai_to_world @ np.linalg.inv(hesai_to_base)
        
        # Mark that we have valid odometry
        self.has_valid_odometry = True
        
        # Publish the robot's transform for visualization
        self.publish_robot_tf(odom_msg.header.stamp)    
    
    def publish_robot_tf(self, stamp):
        """Publish the robot's transform from world to base"""
        t = self.robot_pose_world[:3, 3]
        q = tf_transformations.quaternion_from_matrix(self.robot_pose_world)
        
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base'
        transform.transform.translation.x = t[0]
        transform.transform.translation.y = t[1]
        transform.transform.translation.z = t[2]
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Create and publish TF message
        tf_msg = TFMessage()
        tf_msg.transforms = [transform]
        self.tf_pub.publish(tf_msg)
    
    def process_pointcloud(self, cloud_msg, source_frame):
        """Process and transform a point cloud to world frame with density filtering"""
        # Skip if we don't have valid odometry yet
        if not self.has_valid_odometry:
            self.get_logger().info('Skipping point cloud - waiting for valid odometry')
            return None
            
        # Extract points
        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if not points:
            return None
        
        points = np.array(points)
        
        # Get transform from sensor to base
        transform_to_base = self.get_transform_chain('base', source_frame, cloud_msg.header.stamp)
        if transform_to_base is None:
            self.get_logger().warn(f'Could not find transform from {source_frame} to base')
            return None
        
        # Extract rotation and translation for sensor to base transform
        q = [transform_to_base.transform.rotation.x, transform_to_base.transform.rotation.y,
             transform_to_base.transform.rotation.z, transform_to_base.transform.rotation.w]
        t = np.array([transform_to_base.transform.translation.x,
                      transform_to_base.transform.translation.y,
                      transform_to_base.transform.translation.z])
        
        # Apply sensor-to-base transform
        R_sensor_to_base = tf_transformations.quaternion_matrix(q)[:3, :3]
        points_in_base = (R_sensor_to_base @ points.T).T + t
        
        # Now transform from base to world using robot's pose
        R_base_to_world = self.robot_pose_world[:3, :3]
        t_base_to_world = self.robot_pose_world[:3, 3]
        
        points_in_world = (R_base_to_world @ points_in_base.T).T + t_base_to_world
        
        # Apply density filtering first to remove sparse noise
        points_in_world = self.apply_density_filter(points_in_world, radius=0.3, min_neighbors=4)
        
        # Apply voxel grid filtering for uniform density
        points_in_world = self.apply_voxel_filter(points_in_world, voxel_size=0.05)
        
        # Create point cloud message in world frame
        header = cloud_msg.header
        header.frame_id = 'world'
        cloud_world = pc2.create_cloud_xyz32(header, points_in_world)
        
        return cloud_world
    
    def apply_density_filter(self, points, radius=0.5, min_neighbors=3):
        """
        Filter out isolated points based on local point density
        
        Args:
            points: Nx3 numpy array of point coordinates
            radius: Search radius for neighbors
            min_neighbors: Minimum number of neighbors required
            
        Returns:
            Filtered point cloud with sparse points removed
        """
        # Skip if too few points
        if len(points) < 100:
            return points
            
        # Build KD-tree for efficient neighbor search
        tree = cKDTree(points)
        
        # Count neighbors for each point
        neighbor_counts = tree.query_ball_point(points, radius, return_length=True)
        
        # Keep only points with sufficient neighbors
        dense_mask = neighbor_counts >= min_neighbors
        
        # Report filtering statistics
        filtered_points = points[dense_mask]
        removed_pct = 100 * (1 - len(filtered_points) / len(points))
        if removed_pct > 0:
            self.get_logger().debug(f'Density filter removed {removed_pct:.1f}% of points')
        
        return filtered_points
    
    def apply_voxel_filter(self, points, voxel_size=0.05):
        """
        Apply voxel grid filtering to create a more uniform point distribution
        
        Args:
            points: Nx3 numpy array of point coordinates
            voxel_size: Size of voxel grid cells (in meters)
            
        Returns:
            Downsampled point cloud with more uniform density
        """
        # Skip if too few points
        if len(points) < 100:
            return points
            
        # Convert to Open3D format
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Apply voxel grid filter
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
        # Convert back to numpy array
        filtered_points = np.asarray(downsampled_pcd.points)
        
        return filtered_points
    
    def process_bag(self):
        """Process the rosbag"""
        self.get_logger().info(f'Processing bag: {self.bag_path}')
        
        # Open the bag file
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
        
        while reader.has_next():
            (topic, data, timestamp_ns) = reader.read_next()
            
            # Create ROS timestamp
            stamp = rclpy.time.Time(nanoseconds=timestamp_ns).to_msg()
            
            # Process TF messages
            if topic == '/tf_static':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=True)
                self.tf_static_pub.publish(msg)
                
            elif topic == '/tf':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=False)
                
            # Process DLIO odometry only
            elif topic == '/boxi/dlio/lidar_map_odometry':
                msg_type = get_message(type_map[topic])
                odom_msg = deserialize_message(data, msg_type)
                self.update_robot_pose(odom_msg)
                
            # Process point clouds
            elif topic in ['/anymal/velodyne/points_undistorted',
                          '/boxi/dlio/hesai_points_undistorted',
                          '/boxi/livox/points_undistorted']:
                
                msg_type = get_message(type_map[topic])
                cloud_msg = deserialize_message(data, msg_type)
                
                # Determine source frame
                if 'velodyne' in topic:
                    source_frame = 'velodyne_lidar'
                    publisher = self.velodyne_pub
                elif 'hesai' in topic:
                    source_frame = 'hesai_lidar'
                    publisher = self.hesai_pub
                elif 'livox' in topic:
                    source_frame = 'livox_lidar'
                    publisher = self.livox_pub
                else:
                    continue
                
                # Transform to world frame
                cloud_world = self.process_pointcloud(cloud_msg, source_frame)
                if cloud_world:
                    publisher.publish(cloud_world)
                
                frame_count += 1
                if frame_count % 10 == 0:
                    self.get_logger().info(f'Processed {frame_count} frames')
                
                # Small delay for visualization
                time.sleep(0.01)
        
        self.get_logger().info(f'Finished processing {frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisNode()
    
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