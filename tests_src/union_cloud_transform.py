#!/usr/bin/env python3
import sys
import os
import shutil
import argparse
import struct
import numpy as np

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader, SequentialWriter, TopicMetadata
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import array

def merge_pointclouds(pc1, pc2, frame_id='map'):
    """
    Merge two PointCloud2 messages into a single one.
    This mimics the C++ code: *laserCloudFullVeloRes = *laserCloudFullVeloRes + *laserCloudFullHoriRes;
    """
    if pc1 is None or len(pc1.data) == 0:
        return pc2
    if pc2 is None or len(pc2.data) == 0:
        return pc1
    
    # Create merged cloud
    merged = PointCloud2()
    merged.header = pc1.header
    merged.header.frame_id = frame_id
    
    # Assuming both clouds have the same fields structure
    merged.fields = pc1.fields
    merged.height = 1  # Unorganized cloud
    merged.width = pc1.width + pc2.width
    merged.point_step = pc1.point_step
    merged.row_step = merged.width * merged.point_step
    merged.is_dense = pc1.is_dense and pc2.is_dense
    merged.is_bigendian = pc1.is_bigendian
    
    # Concatenate the data
    merged.data = bytes(pc1.data) + bytes(pc2.data)
    
    return merged

def extract_union_cloud_fields(data, timestamp):
    """
    Manually parse the union_cloud message structure to extract embedded PointCloud2 messages.
    Based on the union_cloud.msg definition:
    - Header header
    - PointCloud2 velo_time_aligned
    - PointCloud2 velo_corner
    - PointCloud2 velo_surface
    - PointCloud2 velo_combine
    - int64 velo_corner_num
    - int64 velo_surf_num
    - CustomMsg livox_time_aligned
    - PointCloud2 livox_corner
    - PointCloud2 livox_surface
    - PointCloud2 livox_combine
    - int64 livox_corner_num
    - int64 livox_surf_num
    - bool livo_degenerated_env
    """
    extracted_clouds = {}
    
    try:
        # Skip the CDR header (4 bytes)
        offset = 4
        
        # Skip Header (variable size)
        # Header has: stamp (8 bytes), frame_id (string with 4-byte length prefix)
        # Skip timestamp
        offset += 8
        
        # Read frame_id length
        frame_id_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4 + frame_id_len
        
        # Align to 4-byte boundary
        if offset % 4 != 0:
            offset += 4 - (offset % 4)
        
        # Now we should be at the first PointCloud2 (velo_time_aligned)
        # We'll try to find PointCloud2 messages by looking for their signature
        
        # PointCloud2 structure:
        # - Header header
        # - uint32 height
        # - uint32 width
        # - PointField[] fields
        # - bool is_bigendian
        # - uint32 point_step
        # - uint32 row_step
        # - uint8[] data
        # - bool is_dense
        
        # Try to find and extract recognizable PointCloud2 patterns
        pointcloud_fields = [
            'velo_combine',
            'livox_combine',
            'velo_corner',
            'velo_surface',
            'livox_corner',
            'livox_surface'
        ]
        
        # Look for PointCloud2 signatures in the data
        # A typical signature would be height=1 for unorganized clouds
        search_offset = offset
        found_clouds = []
        
        while search_offset < len(data) - 100:
            # Look for potential PointCloud2 header
            # Check for height=1 and reasonable width value
            try:
                # Skip potential header
                test_offset = search_offset
                
                # Skip timestamp (8 bytes)
                test_offset += 8
                
                # Skip frame_id
                if test_offset + 4 <= len(data):
                    fid_len = struct.unpack_from('<I', data, test_offset)[0]
                    if fid_len > 0 and fid_len < 256:  # Reasonable frame_id length
                        test_offset += 4 + fid_len
                        
                        # Align
                        if test_offset % 4 != 0:
                            test_offset += 4 - (test_offset % 4)
                        
                        # Check height and width
                        if test_offset + 8 <= len(data):
                            height = struct.unpack_from('<I', data, test_offset)[0]
                            width = struct.unpack_from('<I', data, test_offset + 4)[0]
                            
                            # Check if this looks like a valid point cloud
                            if height == 1 and width > 0 and width < 1000000:
                                # Try to extract this as a PointCloud2
                                pc2_data = data[search_offset:]
                                try:
                                    pc_msg = deserialize_message(pc2_data, PointCloud2)
                                    if pc_msg and pc_msg.width > 0:
                                        found_clouds.append((search_offset, pc_msg))
                                        # Jump ahead past this cloud
                                        search_offset = test_offset + 8 + 4 * len(pc_msg.fields) + pc_msg.row_step + 100
                                        continue
                                except:
                                    pass
            except:
                pass
            
            search_offset += 1
        
        # Assign found clouds to fields
        if len(found_clouds) >= 2:
            # At minimum we should have velo_combine and livox_combine
            extracted_clouds['velo_combine'] = found_clouds[0][1]
            extracted_clouds['livox_combine'] = found_clouds[1][1]
            
            if len(found_clouds) >= 4:
                extracted_clouds['velo_corner'] = found_clouds[2][1]
                extracted_clouds['velo_surface'] = found_clouds[3][1]
            
            if len(found_clouds) >= 6:
                extracted_clouds['livox_corner'] = found_clouds[4][1]
                extracted_clouds['livox_surface'] = found_clouds[5][1]
        
    except Exception as e:
        print(f"Error extracting union_cloud fields: {e}")
    
    return extracted_clouds

def process_union_clouds(input_bag_path, output_bag_path, target_frame):
    """
    Process the bag and extract/merge union_cloud messages into standard PointCloud2 messages.
    """
    print(f"Processing bag: {input_bag_path}")
    print("Extracting and merging union_cloud data...")
    
    # Remove output directory if it exists
    if os.path.exists(output_bag_path):
        print(f"Output directory '{output_bag_path}' already exists. Removing it.")
        shutil.rmtree(output_bag_path)
    
    # Open input bag
    storage_options_in = StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options_in, converter_options)
    
    # Open output bag
    storage_options_out = StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    writer = SequentialWriter()
    writer.open(storage_options_out, converter_options)
    
    # Get all topics
    all_topics_and_types = reader.get_all_topics_and_types()
    
    # Find all topics to process
    union_cloud_topics = []
    standard_pc2_topics = []
    
    for topic in all_topics_and_types:
        if 'union_cloud' in topic.type:
            union_cloud_topics.append(topic.name)
            print(f"  Found union_cloud topic: {topic.name}")
        elif 'sensor_msgs/msg/PointCloud2' in topic.type:
            standard_pc2_topics.append(topic.name)
    
    # Define output topics
    output_topics = {
        '/merged_cloud': 'sensor_msgs/msg/PointCloud2',  # Velo + Livox combined
        '/velo_only': 'sensor_msgs/msg/PointCloud2',     # Velodyne combine
        '/livox_only': 'sensor_msgs/msg/PointCloud2',    # Livox combine
    }
    
    # Also copy standard topics
    for topic_name in standard_pc2_topics:
        output_topics[topic_name] = 'sensor_msgs/msg/PointCloud2'
    
    # Create topics in output bag
    topic_id = 0
    for topic_name, topic_type in output_topics.items():
        topic_metadata = TopicMetadata(
            id=topic_id,
            name=topic_name,
            type=topic_type,
            serialization_format='cdr',
            offered_qos_profiles=[],
            type_description_hash=''
        )
        writer.create_topic(topic_metadata)
        topic_id += 1
    
    # Process messages
    print("\nProcessing messages...")
    message_counts = {name: 0 for name in output_topics.keys()}
    total_processed = 0
    
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        
        if topic_name in union_cloud_topics:
            # Extract embedded PointCloud2 messages from union_cloud
            extracted = extract_union_cloud_fields(data, timestamp)
            
            if extracted:
                # Write individual clouds
                if 'velo_combine' in extracted:
                    velo_cloud = extracted['velo_combine']
                    velo_cloud.header.frame_id = target_frame
                    writer.write('/velo_only', serialize_message(velo_cloud), timestamp)
                    message_counts['/velo_only'] += 1
                
                if 'livox_combine' in extracted:
                    livox_cloud = extracted['livox_combine']
                    livox_cloud.header.frame_id = target_frame
                    writer.write('/livox_only', serialize_message(livox_cloud), timestamp)
                    message_counts['/livox_only'] += 1
                
                # Create merged cloud (Velo + Livox)
                if 'velo_combine' in extracted and 'livox_combine' in extracted:
                    merged = merge_pointclouds(
                        extracted['velo_combine'],
                        extracted['livox_combine'],
                        target_frame
                    )
                    writer.write('/merged_cloud', serialize_message(merged), timestamp)
                    message_counts['/merged_cloud'] += 1
                
                total_processed += 1
                if total_processed % 10 == 0:
                    print(f"  Processed {total_processed} union_cloud messages...")
        
        elif topic_name in standard_pc2_topics:
            # Copy standard PointCloud2 topics
            try:
                msg = deserialize_message(data, PointCloud2)
                msg.header.frame_id = target_frame
                writer.write(topic_name, serialize_message(msg), timestamp)
                message_counts[topic_name] += 1
            except Exception as e:
                print(f"Error processing {topic_name}: {e}")
    
    # Close bags
    reader.close()
    writer.close()
    
    # Summary
    print('\n--- Processing Complete ---')
    print('Created topics:')
    for topic, count in message_counts.items():
        if count > 0:
            print(f'  {topic}: {count} messages')
    print(f'\nOutput bag saved to: {output_bag_path}')
    print('\nTo visualize in RViz2:')
    print('  1. ros2 bag play', output_bag_path)
    print('  2. In RViz2, add PointCloud2 displays for:')
    print('     - /merged_cloud (combined Velodyne + Livox)')
    print('     - /velo_only (Velodyne only)')
    print('     - /livox_only (Livox only)')
    print(f'  3. Set Fixed Frame to "{target_frame}"')

def main():
    parser = argparse.ArgumentParser(
        description="Extract and merge union_cloud messages into standard PointCloud2 format.")
    parser.add_argument('--input_bag', type=str, 
                        default='~/Desktop/KIT_robotics/RangeLDM/mmloam_ros2',
                        help='Path to the input ROS2 bag directory.')
    parser.add_argument('--output_bag', type=str, 
                        default='~/Desktop/KIT_robotics/RangeLDM/merged_clouds',
                        help='Path for the output ROS2 bag directory.')
    parser.add_argument('--target_frame', type=str, default='map',
                        help='The frame_id to assign to the extracted point clouds.')
    
    args = parser.parse_args()
    
    # Expand paths
    input_path = os.path.expanduser(args.input_bag)
    output_path = os.path.expanduser(args.output_bag)
    
    # Check if input exists
    if not os.path.exists(input_path):
        print(f"Error: Input bag not found at {input_path}")
        return
    
    process_union_clouds(input_path, output_path, args.target_frame)

if __name__ == '__main__':
    main()