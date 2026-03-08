"""
Data loader for Grand Tour dataset (ANYmal robot with Hesai LiDAR).
Reads ROS2 bags containing sensor_msgs/PointCloud2 messages.
"""
from collections import defaultdict
from glob import glob
import os
import struct
from typing import Any
from .dataset import RangeDataset, RangeLoader, point_cloud_to_range_image
import numpy as np
import pytorch_lightning as pl
from torch.utils.data import DataLoader, Dataset
from pathlib import Path
import torch
from typing import Optional, Tuple

# Use rosbags library for reading ROS2 bags without ROS2 installation
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


class point_cloud_to_range_image_GrandTour(point_cloud_to_range_image):
    """
    Range image converter for Hesai LiDAR (Pandar XT-32 or similar).
    The Hesai sensors typically have 32 beams with specific vertical angles.
    """
    def __init__(self, 
                 num_beams=32,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.H = num_beams
        
        # Hesai Pandar XT-32 beam angles (approximate values in degrees)
        # These are typical values - adjust based on your specific sensor
        if num_beams == 32:
            # Hesai XT-32 has a vertical FOV of about 31 degrees (-16 to +15)
            self.zenith = np.linspace(15, -16, num_beams).astype(np.float32)
        elif num_beams == 64:
            # For 64-beam sensors
            self.zenith = np.linspace(15, -25, num_beams).astype(np.float32)
        elif num_beams == 128:
            # For 128-beam sensors
            self.zenith = np.linspace(15, -25, num_beams).astype(np.float32)
        else:
            # Default: evenly distributed beams
            self.zenith = np.linspace(15, -15, num_beams).astype(np.float32)
        
        # Convert to radians
        self.zenith = np.radians(self.zenith)
        
        # For Hesai, we don't have height offset like KITTI
        self.height = np.zeros_like(self.zenith)
        self.incl = -self.zenith
        
        # Flag to indicate if we've auto-detected beam configuration
        self.calibration_initialized = False
    
    def auto_detect_beams(self, points):
        """
        Auto-detect the number of beams and their angles from point cloud data.
        This is useful when the exact sensor configuration is unknown.
        """
        if len(points) == 0:
            return
            
        # Calculate vertical angles for all points
        xy_norm = np.linalg.norm(points[:, :2], ord=2, axis=1)
        vert_angles = np.arctan2(points[:, 2], xy_norm)
        
        # Find unique beam angles using histogram
        hist, bin_edges = np.histogram(vert_angles, bins=200)
        
        # Find peaks in the histogram (these correspond to beam angles)
        peak_threshold = np.max(hist) * 0.1
        peak_indices = np.where(hist > peak_threshold)[0]
        
        if len(peak_indices) > 0:
            # Get the beam angles from the bin centers
            detected_angles = (bin_edges[peak_indices] + bin_edges[peak_indices + 1]) / 2
            detected_angles = np.sort(detected_angles)[::-1]  # Sort descending
            
            if len(detected_angles) >= 16:  # Minimum reasonable beam count
                self.H = len(detected_angles)
                self.zenith = -detected_angles.astype(np.float32)
                self.height = np.zeros_like(self.zenith)
                self.incl = -self.zenith
                print(f"Auto-detected {self.H} beams")
        
        self.calibration_initialized = True
    
    def get_row_inds(self, pc):
        """
        Determine row indices for each point based on vertical angle.
        """
        # Calculate vertical angle for each point
        xy_norm = np.linalg.norm(pc[:, :2], ord=2, axis=1)
        point_vert_angles = np.arctan2(pc[:, 2], xy_norm)
        
        # Find the closest beam angle for each point
        error_list = []
        for i in range(len(self.incl)):
            error = np.abs(self.incl[i] - point_vert_angles)
            error_list.append(error)
        all_error = np.stack(error_list, axis=-1)
        row_inds = np.argmin(all_error, axis=-1)
        return row_inds


def parse_pointcloud2(msg) -> np.ndarray:
    """
    Parse a sensor_msgs/PointCloud2 message into a numpy array of points.
    Returns array of shape (N, 4) with columns [x, y, z, intensity].
    """
    # Get point cloud dimensions
    width = msg.width
    height = msg.height
    point_step = msg.point_step
    row_step = msg.row_step
    data = bytes(msg.data)
    
    # Parse field information
    fields = {}
    for field in msg.fields:
        fields[field.name] = {
            'offset': field.offset,
            'datatype': field.datatype,
            'count': field.count
        }
    
    # Datatype mapping (from sensor_msgs/PointField)
    dtype_map = {
        1: ('b', 1),   # INT8
        2: ('B', 1),   # UINT8
        3: ('h', 2),   # INT16
        4: ('H', 2),   # UINT16
        5: ('i', 4),   # INT32
        6: ('I', 4),   # UINT32
        7: ('f', 4),   # FLOAT32
        8: ('d', 8),   # FLOAT64
    }
    
    # Get field specifications
    x_spec = fields.get('x', fields.get('X'))
    y_spec = fields.get('y', fields.get('Y'))
    z_spec = fields.get('z', fields.get('Z'))
    intensity_spec = fields.get('intensity', fields.get('i', fields.get('reflectivity')))
    
    if x_spec is None or y_spec is None or z_spec is None:
        raise ValueError("PointCloud2 message missing x, y, or z fields")
    
    # Calculate number of points
    num_points = width * height
    
    # Pre-allocate output array
    points = np.zeros((num_points, 4), dtype=np.float32)
    
    # Parse points
    for i in range(num_points):
        offset = i * point_step
        
        # Parse x, y, z
        x_fmt, x_size = dtype_map[x_spec['datatype']]
        y_fmt, y_size = dtype_map[y_spec['datatype']]
        z_fmt, z_size = dtype_map[z_spec['datatype']]
        
        x = struct.unpack_from(x_fmt, data, offset + x_spec['offset'])[0]
        y = struct.unpack_from(y_fmt, data, offset + y_spec['offset'])[0]
        z = struct.unpack_from(z_fmt, data, offset + z_spec['offset'])[0]
        
        # Parse intensity if available
        if intensity_spec is not None:
            int_fmt, int_size = dtype_map[intensity_spec['datatype']]
            intensity = struct.unpack_from(int_fmt, data, offset + intensity_spec['offset'])[0]
        else:
            intensity = 0.0
        
        points[i] = [x, y, z, intensity]
    
    # Filter out invalid points (NaN or zero range)
    valid_mask = np.isfinite(points).all(axis=1)
    valid_mask &= (np.linalg.norm(points[:, :3], axis=1) > 0.1)  # Remove points too close to origin
    
    return points[valid_mask]


def parse_pointcloud2_fast(msg) -> np.ndarray:
    """
    Fast parsing of PointCloud2 message using numpy structured arrays.
    Returns array of shape (N, 4) with columns [x, y, z, intensity].
    """
    # Get point cloud dimensions
    width = msg.width
    height = msg.height
    point_step = msg.point_step
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    
    # Build numpy dtype from fields
    fields = {}
    for field in msg.fields:
        fields[field.name] = {
            'offset': field.offset,
            'datatype': field.datatype,
        }
    
    # Datatype mapping
    np_dtype_map = {
        1: np.int8,
        2: np.uint8,
        3: np.int16,
        4: np.uint16,
        5: np.int32,
        6: np.uint32,
        7: np.float32,
        8: np.float64,
    }
    
    num_points = width * height
    
    # Reshape data to access individual points
    point_data = data[:num_points * point_step].reshape(num_points, point_step)
    
    # Extract x, y, z
    x_spec = fields.get('x', fields.get('X'))
    y_spec = fields.get('y', fields.get('Y'))
    z_spec = fields.get('z', fields.get('Z'))
    
    x = point_data[:, x_spec['offset']:x_spec['offset']+4].view(np_dtype_map[x_spec['datatype']]).flatten()
    y = point_data[:, y_spec['offset']:y_spec['offset']+4].view(np_dtype_map[y_spec['datatype']]).flatten()
    z = point_data[:, z_spec['offset']:z_spec['offset']+4].view(np_dtype_map[z_spec['datatype']]).flatten()
    
    # Extract intensity if available
    intensity_spec = fields.get('intensity', fields.get('i', fields.get('reflectivity')))
    if intensity_spec is not None:
        dtype = np_dtype_map[intensity_spec['datatype']]
        dtype_size = np.dtype(dtype).itemsize
        intensity = point_data[:, intensity_spec['offset']:intensity_spec['offset']+dtype_size].view(dtype).flatten().astype(np.float32)
    else:
        intensity = np.zeros(num_points, dtype=np.float32)
    
    # Stack into (N, 4) array
    points = np.stack([x, y, z, intensity], axis=1).astype(np.float32)
    
    # Filter out invalid points
    valid_mask = np.isfinite(points).all(axis=1)
    valid_mask &= (np.linalg.norm(points[:, :3], axis=1) > 0.1)
    
    return points[valid_mask]


class GrandTourRangeDataset(RangeDataset):
    """
    Dataset for Grand Tour ANYmal robot data with Hesai LiDAR.
    Reads ROS2 bags and converts point clouds to range images.
    """
    def __init__(self, 
                 grand_tour_path, 
                 train=True, 
                 width=1024, 
                 grid_sizes=[1, 1024, 1024], 
                 pc_range=[-25.6, -25.6, -3., 25.6, 25.6, 1.], 
                 log=False,
                 inverse=False,
                 split_ratio=0.9,
                 topic_name='/boxi/hesai/points_undistorted',
                 num_beams=32,
                 **kwargs):
        super().__init__(**kwargs)
        
        self.topic_name = topic_name
        self.grand_tour_path = grand_tour_path
        
        # Find all ROS2 bag directories (they contain metadata.yaml)
        bag_dirs = []
        for root, dirs, files in os.walk(grand_tour_path):
            if 'metadata.yaml' in files:
                bag_dirs.append(root)
        
        if len(bag_dirs) == 0:
            raise RuntimeError(f"No ROS2 bag directories found in {grand_tour_path}")
        
        print(f"Found {len(bag_dirs)} ROS2 bag directories")
        
        # Build index of all messages across all bags
        all_messages = []
        
        for bag_dir in sorted(bag_dirs):
            try:
                with Reader(bag_dir) as reader:
                    # Find the point cloud topic
                    topic_found = False
                    for connection in reader.connections:
                        if connection.topic == topic_name:
                            topic_found = True
                            break
                    
                    if not topic_found:
                        print(f"Warning: Topic '{topic_name}' not found in {bag_dir}")
                        continue
                    
                    # Count messages for this topic
                    msg_count = 0
                    for connection, timestamp, rawdata in reader.messages():
                        if connection.topic == topic_name:
                            all_messages.append((bag_dir, msg_count, timestamp))
                            msg_count += 1
                    
                    print(f"Found {msg_count} messages in {os.path.basename(bag_dir)}")
                    
            except Exception as e:
                print(f"Error reading bag {bag_dir}: {e}")
                continue
        
        if len(all_messages) == 0:
            raise RuntimeError(f"No point cloud messages found in any bag")
        
        print(f"Total messages across all bags: {len(all_messages)}")
        
        # Apply train/test split
        total_messages = len(all_messages)
        split_idx = max(1, min(total_messages - 1, int(total_messages * split_ratio)))
        
        if total_messages > 1:
            if train:
                self.messages = all_messages[:split_idx]
            else:
                self.messages = all_messages[split_idx:]
        else:
            self.messages = all_messages
        
        print(f"Dataset split: {len(self.messages)} messages for {'training' if train else 'testing'}")
        
        # Initialize the range image converter
        self.to_range_image = point_cloud_to_range_image_GrandTour(
            num_beams=num_beams,
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse
        )
        
        # Cache for open bag readers (to avoid reopening bags repeatedly)
        self._reader_cache = {}
    
    def __len__(self):
        return len(self.messages)
    
    def _get_message_data(self, bag_dir, msg_idx):
        """Read a specific message from a bag file."""
        with Reader(bag_dir) as reader:
            current_idx = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == self.topic_name:
                    if current_idx == msg_idx:
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        return msg
                    current_idx += 1
        return None
    
    def get_pts(self, message_info):
        """Extract point cloud from ROS2 bag message."""
        bag_dir, msg_idx, timestamp = message_info
        
        msg = self._get_message_data(bag_dir, msg_idx)
        if msg is None:
            return np.zeros((0, 4), dtype=np.float32)
        
        try:
            points = parse_pointcloud2_fast(msg)
            
            # Auto-detect beam configuration from first point cloud
            if not self.to_range_image.calibration_initialized and len(points) > 1000:
                self.to_range_image.auto_detect_beams(points)
            
            return points
        except Exception as e:
            print(f"Error parsing point cloud: {e}")
            return np.zeros((0, 4), dtype=np.float32)
    
    def get_pth_path(self, message_info):
        """Generate cache path for processed range images."""
        bag_dir, msg_idx, timestamp = message_info
        
        # Create cache directory alongside the bag
        cache_dir = os.path.join(bag_dir, 'range_cache')
        os.makedirs(cache_dir, exist_ok=True)
        
        # Use timestamp and index for unique filename
        basename = f"frame_{msg_idx:06d}_{timestamp}.pth"
        return os.path.join(cache_dir, basename)
    
    def __getitem__(self, idx):
        message_info = self.messages[idx]
        pth_path = self.get_pth_path(message_info)
        
        if os.path.exists(pth_path):
            ret = torch.load(pth_path)
        else:
            pts = self.get_pts(message_info)
            
            if len(pts) == 0:
                # Return empty range image if no points
                range_image = np.full((self.to_range_image.H, self.to_range_image.width, 2), 
                                      self.to_range_image.range_fill_value, dtype=np.float32)
            else:
                range_image = self.to_range_image(pts)
            
            range_image, range_image_mask, car_window_mask = self.to_range_image.process_miss_value(range_image)
            range_image = self.to_range_image.normalize(range_image)
            range_image = torch.from_numpy(range_image).permute(2, 1, 0)  # (C, W, H)
            
            ret = {'jpg': range_image}
            
            # Save to cache
            try:
                torch.save(ret, pth_path)
            except Exception as e:
                print(f"Warning: Could not save cache file: {e}")
        
        return ret


class GrandTourRangeLoader(RangeLoader):
    """
    PyTorch Lightning DataModule for Grand Tour dataset.
    """
    def __init__(self, 
                 grand_tour_path, 
                 used_feature=2, 
                 width=1024, 
                 grid_sizes=[1, 1024, 1024], 
                 pc_range=[-25.6, -25.6, -3., 25.6, 25.6, 1.], 
                 log=False, 
                 inverse=False, 
                 downsample=None,
                 inpainting=None,
                 coord=False,
                 topic_name='/boxi/hesai/points_undistorted',
                 num_beams=32,
                 **kwargs):
        super().__init__(**kwargs)
        
        train_dataset = GrandTourRangeDataset(
            grand_tour_path, 
            train=True, 
            used_feature=used_feature, 
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse,
            downsample=downsample,
            inpainting=inpainting,
            coord=coord,
            topic_name=topic_name,
            num_beams=num_beams
        )
        
        test_dataset = GrandTourRangeDataset(
            grand_tour_path, 
            train=False, 
            used_feature=used_feature, 
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse,
            downsample=downsample,
            inpainting=inpainting,
            coord=coord,
            topic_name=topic_name,
            num_beams=num_beams
        )
        
        self.train_dataset = train_dataset
        self.test_dataset = test_dataset


if __name__ == "__main__":
    import argparse
    from PIL import Image

    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, help="Grand Tour dataset root folder")
    parser.add_argument("--save", default=None, help="Optional PNG output path")
    parser.add_argument("--topic", default="/boxi/hesai/points_undistorted", 
                        help="ROS2 topic name for point cloud")
    args = parser.parse_args()

    # Build a tiny loader (one sample, CPU)
    loader = GrandTourRangeLoader(
        grand_tour_path=args.root,
        batch_size=1,
        num_workers=0,
        shuffle=False,
        topic_name=args.topic,
    )
    
    sample = next(iter(loader.test_dataloader()))
    rng = sample["jpg"][0]  # (C, W, H)

    print("Range-image tensor shape:", rng.shape)

    if args.save:
        # Take only channel-0 (range), un-normalize, map to 0-255 for display
        ch0 = rng[0] * loader.test_dataset.to_range_image.std \
                       + loader.test_dataset.to_range_image.mean
        ch0 = ch0.cpu().numpy() 
        img = (ch0 / ch0.max() * 255).astype("uint8").T  # H×W
        Image.fromarray(img).save(args.save)
        print("Saved PNG to", args.save)

        # Save intensity image
        ch1 = rng[1].cpu().numpy()
        ch1_img = (ch1 / ch1.max() * 255).astype(np.uint8).T
        inten_path = args.save.replace(".png", "_intensity.png")
        Image.fromarray(ch1_img).save(inten_path)
        print("Saved intensity PNG to", inten_path)
