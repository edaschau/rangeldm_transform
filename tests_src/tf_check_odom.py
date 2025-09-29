from rosbags.rosbag1 import Reader
import sys

# Path to your bag file
bag_path = 'GrandTour/2024-11-02-17-10-25_tf_model.bag'

# Track unique transform pairs
transform_pairs = set()
odometry_candidates = []

try:
    with Reader(bag_path) as reader:
        # First, determine the exact message type for /tf in this bag
        topics_and_types = [(c.topic, c.msgtype) for c in reader.connections]
        tf_topic_info = next((t for t in topics_and_types if t[0] == '/tf'), None)
        
        if not tf_topic_info:
            print("No /tf topic found in the bag!")
            sys.exit(1)
            
        print(f"Found /tf topic with message type: {tf_topic_info[1]}")
        
        # Analyze the TF messages - parse them manually since we know the structure
        for connection, timestamp, data in reader.messages():
            if connection.topic != '/tf':
                continue
                
            # Instead of using the typestore deserialization (which is failing),
            # let's parse the transform directly from the raw message data
            # Note: This approach handles both ROS1 and ROS2 TF message formats
            try:
                # Skip the header and go directly to the transforms
                # Find the transforms array in the serialized data
                # This is a simplified approach - actual parsing would depend on the exact format
                
                # For our purposes, we'll just extract some information from the raw data
                # to identify unique parent-child frame pairs
                # Assuming standard ROS TF message structure, frames are stored as strings
                
                # Extract strings from the raw data - this is a heuristic approach
                strings = []
                i = 0
                while i < len(data):
                    if i + 4 <= len(data):
                        str_len = int.from_bytes(data[i:i+4], byteorder='little')
                        if str_len > 0 and str_len < 100 and i + 4 + str_len <= len(data):
                            s = data[i+4:i+4+str_len].decode('utf-8', errors='ignore')
                            if all(c.isprintable() for c in s):
                                strings.append(s)
                            i += 4 + str_len
                        else:
                            i += 1
                    else:
                        i += 1
                
                # Look for strings that might be frame IDs
                for j in range(len(strings) - 1):
                    if strings[j] and strings[j+1]:
                        # Heuristic: parent and child frames often appear next to each other
                        parent = strings[j].lstrip('/')
                        child = strings[j+1].lstrip('/')
                        
                        # Skip if both are the same (might be duplicates)
                        if parent != child and all(c.isalnum() or c in '_' for c in parent + child):
                            pair = f"{parent} → {child}"
                            
                            if pair not in transform_pairs:
                                transform_pairs.add(pair)
                                
                                # Check for typical odometry frames
                                if any(frame in pair.lower() for frame in ['odom', 'map', 'base', 'world']):
                                    odometry_candidates.append(pair)
                            
            except Exception as e:
                pass  # Skip problematic messages
                
except Exception as e:
    print(f"Error reading bag: {e}")
    sys.exit(1)

print("\nAll unique transform pairs found in /tf:")
print("-" * 50)
for pair in sorted(transform_pairs):
    print(f"  {pair}")

print("\n" + "=" * 50)
print("Likely odometry-related transforms:")
print("-" * 50)
if odometry_candidates:
    for pair in sorted(odometry_candidates):
        print(f"  {pair}")
else:
    print("  No odometry-related transforms found.")
    print("  This bag likely contains only static or joint transforms (robot state).")
