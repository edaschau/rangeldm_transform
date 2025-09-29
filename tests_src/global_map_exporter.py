#!/usr/bin/env python3
"""
NOT WORKING
Simple Global Map Exporter - Creates a single accumulated point cloud map and exports to a ROS2 bag
"""
import rclpy
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf_transformations
from collections import deque
import time
import argparse
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
import open3d as o3d
import os
from rclpy.time import Time
from rosbag2_py import TopicMetadata

class SimpleMapExporter:
    def __init__(self, input_bag_path, output_bag_path):
        # Paths
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        
        # Global map storage
        self.global_map_points = []
        
        # Transform tracking
        self.tf_buffer = {}
        self.static_tf_buffer = {}
        
        # Odometry tracking
        self.robot_pose_world = np.eye(4)  # Identity matrix - start at origin
        self.has_valid_odometry = False
        
        print("Map Exporter initialized")
    
    def parse_tf_message(self, tf_msg, is_static=False):
        """Store transforms in buffer"""
        for transform in tf_msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            if is_static:
                self.static_tf_buffer[key] = transform
            else:
                if key not in self.tf_buffer:
                    self.tf_buffer[key] = deque(maxlen=1000)
                self.tf_buffer[key].append({
                    'timestamp': transform.header.stamp,
                    'transform': transform
                })
    
    def get_transform_at_time(self, parent_frame, child_frame, timestamp=None):
        """Get transform between frames at specific timestamp"""
        key = (parent_frame, child_frame)
        
        if key in self.static_tf_buffer:
            return self.static_tf_buffer[key]
        
        if key in self.tf_buffer and len(self.tf_buffer[key]) > 0:
            if timestamp is None:
                return self.tf_buffer[key][-1]['transform']
            else:
                for tf_data in reversed(self.tf_buffer[key]):
                    if tf_data['timestamp'].sec <= timestamp.sec and \
                       tf_data['timestamp'].nanosec <= timestamp.nanosec:
                        return tf_data['transform']
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
        t1 = np.array([tf1.transform.translation.x, tf1.transform.translation.y, tf1.transform.translation.z])
        q1 = [tf1.transform.rotation.x, tf1.transform.rotation.y, tf1.transform.rotation.z, tf1.transform.rotation.w]
        
        t2 = np.array([tf2.transform.translation.x, tf2.transform.translation.y, tf2.transform.translation.z])
        q2 = [tf2.transform.rotation.x, tf2.transform.rotation.y, tf2.transform.rotation.z, tf2.transform.rotation.w]
        
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
        
        # Apply 180 degree rotation to correct the flipped orientation
        angle = np.pi  # 180 degrees
        rotation_z_180 = np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle),  np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Calculate the base frame position in world frame
        self.robot_pose_world = rotation_z_180 @ R_hesai_to_world @ np.linalg.inv(hesai_to_base)
        self.has_valid_odometry = True
    
    def process_pointcloud(self, cloud_msg, source_frame):
        """Process and transform a point cloud to world frame"""
        if not self.has_valid_odometry:
            print("Skipping point cloud - waiting for valid odometry")
            return
            
        # Extract points
        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if not points:
            return
        
        points = np.array(points)
        
        # Get transform from sensor to base
        transform_to_base = self.get_transform_chain('base', source_frame, cloud_msg.header.stamp)
        if transform_to_base is None:
            print(f'Could not find transform from {source_frame} to base')
            return
        
        # Extract rotation and translation for sensor to base transform
        q = [transform_to_base.transform.rotation.x, transform_to_base.transform.rotation.y,
             transform_to_base.transform.rotation.z, transform_to_base.transform.rotation.w]
        t = np.array([transform_to_base.transform.translation.x,
                      transform_to_base.transform.translation.y,
                      transform_to_base.transform.translation.z])
        
        # Apply sensor-to-base transform
        R_sensor_to_base = tf_transformations.quaternion_matrix(q)[:3, :3]
        points_in_base = (R_sensor_to_base @ points.T).T + t
        
        # Transform from base to world using robot's pose
        R_base_to_world = self.robot_pose_world[:3, :3]
        t_base_to_world = self.robot_pose_world[:3, 3]
        
        points_in_world = (R_base_to_world @ points_in_base.T).T + t_base_to_world
        
        # Add points to global map
        self.global_map_points.append(points_in_world)
    
    def filter_map(self, voxel_size=0.05, radius=0.3, min_neighbors=4):
        """Filter the global map to remove noise and downsample"""
        if not self.global_map_points:
            return np.zeros((0, 3))
            
        # Combine all point clouds
        all_points = np.vstack(self.global_map_points)
        print(f"Raw global map size: {len(all_points)} points")
        
        # # Density filtering
        # if len(all_points) > 1000:
        #     tree = cKDTree(all_points)
        #     neighbor_counts = tree.query_ball_point(all_points, radius, return_length=True)
        #     dense_mask = neighbor_counts >= min_neighbors
        #     all_points = all_points[dense_mask]
        #     print(f"After density filtering: {len(all_points)} points")
        
        # # Voxel grid filtering
        # if len(all_points) > 1000:
        #     pcd = o3d.geometry.PointCloud()
        #     pcd.points = o3d.utility.Vector3dVector(all_points)
        #     downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        #     all_points = np.asarray(downsampled_pcd.points)
        #     print(f"After voxel filtering: {len(all_points)} points")
        
        return all_points
    
    def create_global_map_message(self, timestamp):
        """Create a PointCloud2 message from the global map"""
        filtered_map = self.filter_map()
        if len(filtered_map) == 0:
            return None
            
        # Create point cloud message
        header = PointCloud2().header
        header.frame_id = 'world'
        header.stamp = timestamp
        
        cloud_msg = pc2.create_cloud_xyz32(header, filtered_map)
        return cloud_msg
    
    def process_bag(self):
        """Process the input bag and write the global map to output bag"""
        print(f'Processing bag: {self.input_bag_path}')
        
        # Open the input bag file
        storage_options_in = rosbag2_py.StorageOptions(
            uri=self.input_bag_path,
            storage_id='sqlite3'
        )
        converter_options_in = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options_in, converter_options_in)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_type.name: topic_type.type for topic_type in topic_types}
        
        # Process all messages to build the map
        frame_count = 0
        last_timestamp_ns = 0
        
        print('Building global map from LiDAR data...')
        while reader.has_next():
            (topic, data, timestamp_ns) = reader.read_next()
            last_timestamp_ns = timestamp_ns
            
            # Create ROS timestamp
            stamp = Time(nanoseconds=timestamp_ns).to_msg()
            
            # Process TF messages
            if topic == '/tf_static':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=True)
                
            elif topic == '/tf':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                self.parse_tf_message(msg, is_static=False)
                
            # Process DLIO odometry
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
                elif 'hesai' in topic:
                    source_frame = 'hesai_lidar'
                elif 'livox' in topic:
                    source_frame = 'livox_lidar'
                else:
                    continue
                
                # Add point cloud to global map
                self.process_pointcloud(cloud_msg, source_frame)
                
                frame_count += 1
                if frame_count % 50 == 0:
                    print(f'Processed {frame_count} point cloud frames')
            if frame_count >= 2000:
                break
        
        print(f'Finished processing {frame_count} frames')
        
        # Create output bag directory if it doesn't exist
        os.makedirs(os.path.dirname(self.output_bag_path), exist_ok=True)
        
        # Open output bag file
        storage_options_out = rosbag2_py.StorageOptions(
            uri=self.output_bag_path,
            storage_id='sqlite3'
        )
        converter_options_out = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options_out, converter_options_out)
        
        # Create the global map topic
        global_map_topic_name = '/global_map'
        global_map_topic_type = 'sensor_msgs/msg/PointCloud2'
        
        # Fixed constructor - need to add id parameter and use empty list for QoS
        global_map_topic = TopicMetadata(
            id=0,  # Add required id parameter
            name=global_map_topic_name,
            type=global_map_topic_type,
            serialization_format='cdr',
            offered_qos_profiles=[]  # Must be a list, not a string
        )
        writer.create_topic(global_map_topic)
        
        # Create and write the global map message
        final_stamp = Time(nanoseconds=last_timestamp_ns).to_msg()
        final_map_msg = self.create_global_map_message(final_stamp)
        
        if final_map_msg:
            global_map_data = serialize_message(final_map_msg)
            writer.write(global_map_topic_name, global_map_data, last_timestamp_ns)
            print(f'Global map with {len(self.filter_map())} points saved to: {self.output_bag_path}')
        else:
            print('No global map created - no valid point clouds found')

def main():
    parser = argparse.ArgumentParser(description='Create and export a global point cloud map.')
    parser.add_argument('--input', type=str, required=True, help='Input bag path')
    parser.add_argument('--output', type=str, required=True, help='Output bag path')
    
    args = parser.parse_args()
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create and run the exporter
    exporter = SimpleMapExporter(
        input_bag_path=args.input,
        output_bag_path=args.output
    )
    
    # Process the bag
    exporter.process_bag()
    
    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()