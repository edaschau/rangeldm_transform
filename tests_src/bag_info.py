# analyze what inside a ros1 bag


from rosbags.rosbag1 import Reader
import os

# Path to your bag file
bag_path = 'GrandTour/2024-11-02-17-10-25_tf_model.bag'

def format_size(size_bytes):
    """Format size in bytes to human-readable format"""
    if size_bytes >= 1024 * 1024:
        return f"{size_bytes / (1024 * 1024):.2f} MB"
    elif size_bytes >= 1024:
        return f"{size_bytes / 1024:.2f} KB"
    else:
        return f"{size_bytes} bytes"

try:
    # Get total file size
    total_file_size = os.path.getsize(bag_path)
    
    # Create a reader instance
    with Reader(bag_path) as reader:
        # Print bag information
        print(f"Bag file: {bag_path}")
        print(f"File size: {format_size(total_file_size)}")
        print(f"Duration: {(reader.end_time - reader.start_time) / 1e9:.2f} seconds")
        
        print("\nMeasuring message sizes...")
        # Initialize topic size tracking
        topic_sizes = {topic: 0 for topic in reader.topics}
        
        # Process all messages to measure sizes
        for connection, timestamp, data in reader.messages():
            topic = connection.topic
            # Use the length of serialized data as message size
            msg_size = len(data)
            topic_sizes[topic] += msg_size
        
        # Get topic statistics with measured sizes
        print("\nTopic Size Statistics:")
        print("=" * 80)
        print(f"{'Topic Name':<40} {'Message Count':<15} {'Data Size':<15} {'Avg Size/Msg':<15}")
        print("-" * 80)
        
        total_measured_size = sum(topic_sizes.values())
        
        # Print statistics for each topic
        for topic, topic_info in reader.topics.items():
            count = topic_info.msgcount
            size = topic_sizes[topic]
            avg_size = size / count if count > 0 else 0
            
            print(f"{topic:<40} {count:<15} {format_size(size):<15} {format_size(avg_size):<15}")
        
        print("-" * 80)
        total_messages = sum(info.msgcount for info in reader.topics.values())
        avg_total = total_measured_size / total_messages if total_messages > 0 else 0
        print(f"{'TOTAL':<40} {total_messages:<15} {format_size(total_measured_size):<15} {format_size(avg_total):<15}")
        
        # Note about measurement
        print(f"\nMeasured data size: {format_size(total_measured_size)}")
        print(f"Total bag file size: {format_size(total_file_size)}")
        print(f"Difference (metadata, indices, etc.): {format_size(total_file_size - total_measured_size)}")
        print(reader.topics)
        
except Exception as e:
    print(f"Error: {e}")