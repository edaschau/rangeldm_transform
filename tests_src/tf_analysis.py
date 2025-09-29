from rosbags.rosbag1 import Reader
import numpy as np
import struct
from collections import defaultdict

# Path to your bag file
bag_path = 'multi-modal-loam/example_output/mm_loam_complete_output.bag'

def extract_pointcloud2_info(data, connection):
    """Extract and return information from a PointCloud2 message"""
    # Parse header
    offset = 0
    seq, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    secs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    nsecs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # Skip frame_id string length and content
    frame_id_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    frame_id = data[offset:offset+frame_id_len].decode('utf-8')
    offset += frame_id_len
    
    # Parse cloud dimensions
    height, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    width, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # Parse fields array length
    fields_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # Parse each field
    fields = []
    for _ in range(fields_len):
        # Parse field name
        name_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        name = data[offset:offset+name_len].decode('utf-8')
        offset += name_len
        
        # Parse other field properties
        field_offset, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        datatype, offset = struct.unpack('<B', data[offset:offset+1])[0], offset+1
        count, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        
        fields.append({
            'name': name,
            'offset': field_offset,
            'datatype': datatype,
            'count': count
        })
    
    # Skip to data section
    is_bigendian, offset = struct.unpack('<B', data[offset:offset+1])[0], offset+1
    point_step, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    row_step, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # Get data length
    data_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # The rest is point data
    point_data = data[offset:offset+data_len]
    offset += data_len
    
    # Get density flag
    is_dense = struct.unpack('<B', data[offset:offset+1])[0]
    
    return {
        'timestamp': f"{secs}.{nsecs:09d}",
        'frame_id': frame_id,
        'dimensions': f"{width}x{height} points",
        'point_count': width * height,
        'fields': fields,
        'point_step': point_step,
        'is_dense': bool(is_dense),
        'data_size': f"{data_len/1024:.2f} KB"
    }

def extract_odometry_info(data):
    """Extract basic information from an Odometry message"""
    # Skip header
    offset = 0
    seq, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    secs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    nsecs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    # Skip frame_id
    frame_id_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    frame_id = data[offset:offset+frame_id_len].decode('utf-8')
    offset += frame_id_len
    
    # Skip child_frame_id
    child_frame_id_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    child_frame_id = data[offset:offset+child_frame_id_len].decode('utf-8')
    offset += child_frame_id_len
    
    # Extract position
    x, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    y, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    z, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    
    # Extract orientation
    qx, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    qy, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    qz, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    qw, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
    
    return {
        'timestamp': f"{secs}.{nsecs:09d}",
        'frame_id': frame_id,
        'child_frame_id': child_frame_id,
        'position': f"x: {x:.3f}, y: {y:.3f}, z: {z:.3f}",
        'orientation': f"qx: {qx:.3f}, qy: {qy:.3f}, qz: {qz:.3f}, qw: {qw:.3f}"
    }

def extract_tf_info(data):
    """Extract basic information from a TFMessage"""
    # Parse the transforms array length
    offset = 0
    transforms_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
    
    transforms = []
    for _ in range(transforms_len):
        # Skip header
        seq, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        secs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        nsecs, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        
        # Get frame_id
        frame_id_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        frame_id = data[offset:offset+frame_id_len].decode('utf-8')
        offset += frame_id_len
        
        # Get child_frame_id
        child_frame_id_len, offset = struct.unpack('<I', data[offset:offset+4])[0], offset+4
        child_frame_id = data[offset:offset+child_frame_id_len].decode('utf-8')
        offset += child_frame_id_len
        
        # Get translation
        tx, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        ty, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        tz, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        
        # Get rotation
        rx, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        ry, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        rz, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        rw, offset = struct.unpack('<d', data[offset:offset+8])[0], offset+8
        
        transforms.append({
            'timestamp': f"{secs}.{nsecs:09d}",
            'parent_frame': frame_id,
            'child_frame': child_frame_id,
            'translation': f"x: {tx:.3f}, y: {ty:.3f}, z: {tz:.3f}",
            'rotation': f"x: {rx:.3f}, y: {ry:.3f}, z: {rz:.3f}, w: {rw:.3f}"
        })
    
    return transforms

try:
    print(f"Analyzing bag: {bag_path}\n")
    
    with Reader(bag_path) as reader:
        # Get summary info
        print(f"Duration: {(reader.end_time - reader.start_time) / 1e9:.2f} seconds")
        
        # Topic stats
        print("\n=== TOPIC STATISTICS ===")
        for topic, info in reader.topics.items():
            print(f"Topic: {topic}")
            print(f"  Type: {info.msgtype}")
            print(f"  Messages: {info.msgcount}")
            print()
        
        # Extract detailed information by message type
        print("\n=== DETAILED ANALYSIS ===")
        
        # Store samples of each message type
        pointcloud_sample = None
        odometry_sample = None
        tf_frames = defaultdict(list)
        lidar_related_tfs = []
        
        # Track point cloud statistics
        pointcloud_sizes = []
        
        # Process a limited number of messages to extract samples
        msg_count = 0
        max_samples = 100  # Limit processing for performance
        
        for connection, timestamp, data in reader.messages():
            msg_count += 1
            if msg_count > max_samples:
                continue
                
            topic = connection.topic
            
            # Get PointCloud2 sample and stats
            if 'hesai_points' in topic and pointcloud_sample is None:
                pointcloud_sample = extract_pointcloud2_info(data, connection)
                pointcloud_sizes.append(pointcloud_sample['point_count'])
            
            # Get Odometry sample
            elif 'odometry' in topic and odometry_sample is None:
                odometry_sample = extract_odometry_info(data)
            
            # Process TF messages to find LiDAR-related transforms
            elif topic == '/tf':
                transforms = extract_tf_info(data)
                for transform in transforms:
                    parent = transform['parent_frame']
                    child = transform['child_frame']
                    tf_frames[(parent, child)].append(transform)
                    
                    # Store any transform related to LiDAR
                    if 'lidar' in parent.lower() or 'lidar' in child.lower() or 'hesai' in parent.lower() or 'hesai' in child.lower():
                        lidar_related_tfs.append(transform)
        
        # Print point cloud details
        if pointcloud_sample:
            print("\n--- POINT CLOUD DETAILS ---")
            print(f"Frame: {pointcloud_sample['frame_id']}")
            print(f"Dimensions: {pointcloud_sample['dimensions']}")
            print(f"Point count: {pointcloud_sample['point_count']}")
            print(f"Data size: {pointcloud_sample['data_size']}")
            print(f"Is dense: {pointcloud_sample['is_dense']}")
            
            print("\nPoint cloud fields:")
            for field in pointcloud_sample['fields']:
                datatype_names = {
                    1: 'INT8', 2: 'UINT8', 3: 'INT16', 4: 'UINT16',
                    5: 'INT32', 6: 'UINT32', 7: 'FLOAT32', 8: 'FLOAT64'
                }
                datatype = datatype_names.get(field['datatype'], f"Unknown({field['datatype']})")
                print(f"  {field['name']}: {datatype} (offset: {field['offset']}, count: {field['count']})")
        
        # Print odometry details
        if odometry_sample:
            print("\n--- ODOMETRY DETAILS ---")
            print(f"Frame: {odometry_sample['frame_id']} -> {odometry_sample['child_frame_id']}")
            print(f"Position: {odometry_sample['position']}")
            print(f"Orientation: {odometry_sample['orientation']}")
        
        # Print TF details related to LiDAR
        if lidar_related_tfs:
            print("\n--- LIDAR-RELATED TRANSFORMS ---")
            for i, tf in enumerate(lidar_related_tfs[:5]):  # Show first 5
                print(f"Transform {i+1}:")
                print(f"  Time: {tf['timestamp']}")
                print(f"  Frames: {tf['parent_frame']} -> {tf['child_frame']}")
                print(f"  Translation: {tf['translation']}")
                print(f"  Rotation: {tf['rotation']}")
            
            if len(lidar_related_tfs) > 5:
                print(f"... and {len(lidar_related_tfs) - 5} more transforms")
        
        # Print all unique frame relationships
        print("\n--- ALL FRAME RELATIONSHIPS ---")
        for (parent, child), _ in list(tf_frames.items())[:20]:  # Limit to first 20
            print(f"{parent} -> {child}")
        
        if len(tf_frames) > 20:
            print(f"... and {len(tf_frames) - 20} more relationships")

except Exception as e:
    print(f"Error: {e}")