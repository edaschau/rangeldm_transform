from rosbags.rosbag1 import Reader

# Path to your bag file
bag_path = 'GrandTour/2024-11-02-17-10-25_dlio.bag'

try:
    # Create a reader instance
    with Reader(bag_path) as reader:
        print(dir(reader))  # List available methods and attributes
        # Print bag information
        print(f"Bag file: {bag_path}")
        print(f"Start time: {reader.start_time}")
        print(f"End time: {reader.end_time}")
        print(f"Duration: {(reader.end_time - reader.start_time) / 1e9:.2f} seconds")
        print(f"{reader.topics}")
        # Get and print topic information
        print("\nTopics in the bag:")
        for topic in reader.topics:
            print(topic)
            
except Exception as e:
    print(f"Error: {e}")