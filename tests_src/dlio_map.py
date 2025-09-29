#!/usr/bin/env python3
"""
Simple DLIO Map Exporter - Creates a global map with minimal overhead
experimantal, unfinished
Includes PCD fallback for reliable output
"""
import rclpy
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf_transformations
import os
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse

class SimpleDlioMapExporter:
    def __init__(self, input_bag, output_bag):
        self.input_bag = input_bag
        self.output_bag = output_bag
        self.points = []
        self.intensities = []
        self.robot_pose = np.eye(4)
        self.has_odom = False
        print(f"Map Exporter initialized: {input_bag} -> {output_bag}")
    
    def process_odometry(self, odom_msg):
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        
        self.robot_pose = tf_transformations.quaternion_matrix(q)
        self.robot_pose[0, 3] = pos.x
        self.robot_pose[1, 3] = pos.y
        self.robot_pose[2, 3] = pos.z
        self.has_odom = True
    
    def process_pointcloud(self, cloud_msg):
        if not self.has_odom:
            return
            
        # Extract points and intensities
        cloud_points = []
        cloud_intensities = []
        
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            cloud_points.append([p[0], p[1], p[2]])
            cloud_intensities.append(p[3])
        
        if not cloud_points:
            return
        
        # Convert to numpy arrays
        cloud_points = np.array(cloud_points)
        cloud_intensities = np.array(cloud_intensities)
        
        # Transform to world frame
        homogeneous_points = np.hstack((cloud_points, np.ones((cloud_points.shape[0], 1))))
        world_points = (self.robot_pose @ homogeneous_points.T).T[:, 0:3]
        
        # Add to map
        self.points.append(world_points)
        self.intensities.append(cloud_intensities)
        
        print(f"\rProcessed {len(self.points)} point clouds...", end="")
    
    def save_as_pcd(self, output_dir):
        """Save the map as PCD file (reliable fallback)"""
        try:
            import open3d as o3d
            
            if not self.points:
                print("No point clouds to save")
                return None
                
            # Combine all points
            combined_points = np.vstack(self.points)
            combined_intensities = np.hstack(self.intensities)
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(combined_points)
            
            # Add colors based on intensity
            if len(combined_intensities) > 0:
                max_i = np.max(combined_intensities)
                if max_i > 0:
                    normalized = combined_intensities / max_i
                    colors = np.zeros((len(combined_points), 3))
                    colors[:, 0] = normalized  # R
                    colors[:, 1] = normalized  # G
                    colors[:, 2] = normalized  # B
                    pcd.colors = o3d.utility.Vector3dVector(colors)
        
            # Save full resolution files
            pcd_path = os.path.join(output_dir, "global_map.pcd")
            o3d.io.write_point_cloud(pcd_path, pcd)
            
            # Also save as PLY (more compatible)
            ply_path = os.path.join(output_dir, "global_map.ply")
            o3d.io.write_point_cloud(ply_path, pcd)
            
            print(f"Saved full resolution point cloud to: {pcd_path}")
            print(f"Also saved as: {ply_path}")
            print(f"Points: {len(combined_points)}")
            return pcd_path
            
        except ImportError:
            print("Warning: Open3D not available, skipping PCD export")
            return None

    def export_to_rosbag(self, output_dir):
        """Try to export the map to a ROS2 bag"""
        try:
            # Create a combined point cloud
            if not self.points:
                print("No point clouds to export")
                return False
            
            combined_points = np.vstack(self.points)
            combined_intensities = np.hstack(self.intensities)
            
            print(f"Creating ROS2 bag with {len(combined_points)} points")
            
            # Add fields for XYZ and intensity
            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1)
            ]
            
            # Try to open the bag with MCAP format first (better for large messages)
            try:
                print("Attempting to write with MCAP format (no downsampling)")
                storage_options = rosbag2_py.StorageOptions(
                    uri=self.output_bag, 
                    storage_id='mcap'
                )
                converter_options = rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'
                )
                
                writer = rosbag2_py.SequentialWriter()
                writer.open(storage_options, converter_options)
                
                # Create topic
                from rosbag2_py import TopicMetadata
                topic = TopicMetadata(
                    name='/global_map',
                    type='sensor_msgs/msg/PointCloud2',
                    serialization_format='cdr',
                    offered_qos_profiles=[],
                    id=0
                )
                writer.create_topic(topic)
                
                # Create the point cloud message without any downsampling
                msg = PointCloud2()
                msg.header.frame_id = 'map'
                cloud = pc2.create_cloud(msg.header, fields, np.column_stack((combined_points, combined_intensities)))
                
                # Write message
                writer.write('/global_map', serialize_message(cloud), 0)
                print(f"Successfully wrote global map using MCAP format with all {len(combined_points)} points")
                return True
                
            except Exception as e:
                print(f"MCAP format failed: {e}")
                print("Falling back to SQLite format with chunking")
                
                # Only downsample for SQLite if necessary
                sqlite_points = combined_points
                sqlite_intensities = combined_intensities
                
                # Check if too large for SQLite
                if len(sqlite_points) > 50000000:  # ~50 million points is around 1GB limit
                    print("Downsampling for SQLite compatibility")
                    idx = np.random.choice(len(sqlite_points), 50000000, replace=False)
                    sqlite_points = sqlite_points[idx]
                    sqlite_intensities = sqlite_intensities[idx]
                
                # Split into chunks for SQLite
                max_points = 15000000  # ~15 million points per chunk should be safe
                num_chunks = (len(sqlite_points) + max_points - 1) // max_points
                
                storage_options = rosbag2_py.StorageOptions(
                    uri=self.output_bag, 
                    storage_id='sqlite3'
                )
                converter_options = rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'
                )
                
                writer = rosbag2_py.SequentialWriter()
                writer.open(storage_options, converter_options)
                
                # Create topic
                from rosbag2_py import TopicMetadata
                topic = TopicMetadata(
                    name='/global_map',
                    type='sensor_msgs/msg/PointCloud2',
                    serialization_format='cdr',
                    offered_qos_profiles=[],
                    id=0
                )
                writer.create_topic(topic)
                
                # Write chunks
                for i in range(num_chunks):
                    start = i * max_points
                    end = min((i + 1) * max_points, len(sqlite_points))
                    
                    chunk_msg = PointCloud2()
                    chunk_msg.header.frame_id = 'map'
                    
                    # Set timestamp
                    from builtin_interfaces.msg import Time
                    stamp = Time()
                    stamp.sec = 0
                    stamp.nanosec = i  # Use chunk number as nanosecs to differentiate chunks
                    chunk_msg.header.stamp = stamp
                    
                    # Create chunk cloud
                    chunk_cloud = pc2.create_cloud(
                        chunk_msg.header, 
                        fields, 
                        np.column_stack((sqlite_points[start:end], sqlite_intensities[start:end]))
                    )
                    
                    # Write chunk
                    writer.write('/global_map', serialize_message(chunk_cloud), i)
                    print(f"Wrote chunk {i+1}/{num_chunks} with {end-start} points")
                
                print("Successfully wrote global map in chunks")
                return True
                
        except Exception as e:
            print(f"Failed to create ROS2 bag: {e}")
            return False

    def run(self):
        """Process the input bag and create global map"""
        # Create output directory
        output_dir = os.path.dirname(self.output_bag)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Open input bag
        storage_options = rosbag2_py.StorageOptions(
            uri=self.input_bag,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        print(f"Reading bag: {self.input_bag}")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_type.name: topic_type.type for topic_type in topic_types}
        
        # Process all messages
        while reader.has_next():
            (topic, data, timestamp_ns) = reader.read_next()
            
            # Process odometry
            if topic == '/boxi/dlio/lidar_map_odometry':
                msg_type = get_message(type_map[topic])
                odom_msg = deserialize_message(data, msg_type)
                self.process_odometry(odom_msg)
            
            # Process point clouds
            elif topic == '/boxi/dlio/hesai_points_undistorted':
                msg_type = get_message(type_map[topic])
                cloud_msg = deserialize_message(data, msg_type)
                self.process_pointcloud(cloud_msg)
        
        print("\nFinished reading bag file")
        
        # Save as PCD first (reliable backup)
        pcd_path = self.save_as_pcd(os.path.dirname(self.output_bag))
        
        # Try exporting to ROS2 bag
        bag_success = self.export_to_rosbag(os.path.dirname(self.output_bag))
        
        if bag_success:
            print("Global map exported successfully as ROS2 bag")
        elif pcd_path:
            print(f"Failed to create ROS2 bag, but saved as PCD file: {pcd_path}")
        else:
            print("Failed to export global map in any format")
            return False
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Simple DLIO Map Exporter')
    parser.add_argument('--input', required=True, help='Input bag path')
    parser.add_argument('--output', required=True, help='Output bag path')
    args = parser.parse_args()
    
    rclpy.init()
    exporter = SimpleDlioMapExporter(args.input, args.output)
    exporter.run()
    rclpy.shutdown()
    print("Done!")

if __name__ == '__main__':
    main()