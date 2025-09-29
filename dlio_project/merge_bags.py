import os
from rosbags.rosbag1 import Reader, Writer

# --- Configuration ---
# NOTE: This script assumes it's run from the 'dlio_project' directory.
data_dir = 'dlio_data'
input_bag_files = [
    os.path.join(data_dir, '2024-11-02-17-10-25_hesai_undist.bag'),
    os.path.join(data_dir, '2024-11-02-17-10-25_stim320_imu.bag'),
    os.path.join(data_dir, '2024-11-02-17-10-25_tf_model.bag'),
]
output_bag_file = os.path.join(data_dir, 'merged_dataset.bag')

# --- Main Script ---
all_messages = []
all_connections = {}

# 1. Read all messages and unique connections from input bags
print("Reading messages and connections from input bags...")
for bag_path in input_bag_files:
    print(f"  - Reading {os.path.basename(bag_path)}")
    with Reader(bag_path) as reader:
        # Store all unique connections by their topic to avoid duplicates
        for conn in reader.connections:
            if conn.topic not in all_connections:
                all_connections[conn.topic] = conn
        
        # Store all messages
        for connection, timestamp, rawdata in reader.messages():
            # We store the original topic name to look it up later
            all_messages.append((connection.topic, timestamp, rawdata))

# 2. Sort all messages chronologically by their timestamp
print("\nSorting all messages by timestamp...")
all_messages.sort(key=lambda msg: msg[1])
print(f"Total messages to write: {len(all_messages)}")

# 3. Write the sorted messages to the new output bag
print(f"\nWriting sorted messages to {os.path.basename(output_bag_file)}...")
with Writer(output_bag_file) as writer:
    # Create a map from topic name to the new connection object in the writer
    connection_map = {}
    for topic, conn in all_connections.items():
        # FIX: As per the documentation, add_connection only takes topic and msgtype.
        new_conn = writer.add_connection(conn.topic, conn.msgtype)
        connection_map[topic] = new_conn

    # Write messages using the new connections
    for topic, timestamp, rawdata in all_messages:
        # Look up the new connection object by topic and write the message
        writer.write(connection_map[topic], timestamp, rawdata)

print("\nDone. Bag merging complete.")