# merge multiple ros2 bags into 1

import argparse
import sys
from pathlib import Path

from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def merge_bags(input_bags, output_bag_path):
    """
    Merges multiple ROS 2 bags into a single bag.

    :param input_bags: A list of paths to the input bag directories.
    :param output_bag_path: The path for the output merged bag directory.
    """
    output_bag_path = Path(output_bag_path)
    if output_bag_path.exists():
        print(f"Error: Output path '{output_bag_path}' already exists.")
        sys.exit(1)

    # --- 1. Collect all topics and messages from all bags ---
    all_messages = []
    all_topics_meta = {}
    print("Reading messages from input bags...")

    for bag_path in input_bags:
        try:
            reader = SequentialReader()
            storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
            converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
            reader.open(storage_options, converter_options)

            # Store topic metadata
            for topic_meta in reader.get_all_topics_and_types():
                if topic_meta.name not in all_topics_meta:
                    all_topics_meta[topic_meta.name] = topic_meta
            
            # Store messages
            while reader.has_next():
                topic, msg, t = reader.read_next()
                all_messages.append((topic, msg, t))
            
            print(f"  - Finished reading from {bag_path}")

        except Exception as e:
            print(f"Error reading bag {bag_path}: {e}")
            sys.exit(1)

    # --- 2. Sort all messages by timestamp ---
    print("Sorting all messages by timestamp...")
    all_messages.sort(key=lambda x: x[2])

    # --- 3. Write sorted messages to the new bag ---
    print(f"Writing {len(all_messages)} messages to '{output_bag_path}'...")
    try:
        writer = SequentialWriter()
        storage_options = StorageOptions(uri=str(output_bag_path), storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        writer.open(storage_options, converter_options)

        # Create topics in the new bag
        for topic_meta in all_topics_meta.values():
            writer.create_topic(topic_meta)

        # Write messages
        for topic, msg, t in all_messages:
            writer.write(topic, msg, t)
            
        print("Merge complete.")

    except Exception as e:
        print(f"Error writing to output bag: {e}")
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Merge multiple ROS 2 bags into one.")
    parser.add_argument(
        '-i', '--input',
        nargs='+',  # Allows for multiple input bags
        required=True,
        help='Paths to the input bag directories to merge.'
    )
    parser.add_argument(
        '-o', '--output',
        required=True,
        help='Path to the output directory for the merged bag.'
    )
    args = parser.parse_args()

    merge_bags(args.input, args.output)