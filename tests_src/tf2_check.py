import argparse
import sys
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def inspect_tf_static(bag_path: str, topic_name: str = '/tf_static'):
    """
    Reads a ROS 2 bag and prints the details of messages on the /tf_static topic.

    :param bag_path: Path to the input bag directory.
    :param topic_name: The static transform topic to inspect.
    """
    bag_file = Path(bag_path)
    if not bag_file.exists() or not bag_file.is_dir():
        print(f"Error: Bag path '{bag_path}' does not exist or is not a directory.")
        sys.exit(1)

    try:
        reader = SequentialReader()
        storage_options = StorageOptions(uri=str(bag_file), storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        sys.exit(1)

    # Get the message type for the target topic from the bag's metadata
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        print(f"Error: Topic '{topic_name}' not found in the bag.")
        print(f"Available topics: {[t for t in topic_types.keys()]}")
        sys.exit(1)
    
    msg_type_str = topic_types[topic_name]
    msg_type = get_message(msg_type_str)

    print(f"Inspecting static transforms on topic '{topic_name}' in bag: {bag_path}\n")
    found_transforms = False

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            found_transforms = True
            # Deserialize the raw data into a ROS message
            msg = deserialize_message(data, msg_type)
            
            if not msg.transforms:
                print(f"Found a TFMessage at time {t} with no transforms.")
                continue

            print(f"--- Found {len(msg.transforms)} transform(s) in a message at timestamp {t} ---")
            for transform_stamped in msg.transforms:
                t = transform_stamped.transform.translation
                r = transform_stamped.transform.rotation
                print(f"  Parent Frame: '{transform_stamped.header.frame_id}'")
                print(f"  Child Frame:  '{transform_stamped.child_frame_id}'")
                print(f"    - Translation: [x: {t.x:.4f}, y: {t.y:.4f}, z: {t.z:.4f}]")
                print(f"    - Rotation (q): [x: {r.x:.4f}, y: {r.y:.4f}, z: {r.z:.4f}, w: {r.w:.4f}]\n")

    if not found_transforms:
        print(f"No messages found on the '{topic_name}' topic.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Inspects /tf_static messages within a ROS 2 bag file."
    )
    parser.add_argument(
        'bag_path',
        type=str,
        help='Path to the ROS 2 bag directory to inspect.'
    )
    parser.add_argument(
        '--topic',
        type=str,
        default='/tf_static',
        help="The static transform topic to inspect (default: /tf_static)."
    )
    args = parser.parse_args()

    inspect_tf_static(args.bag_path, args.topic)