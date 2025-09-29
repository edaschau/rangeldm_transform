from collections import defaultdict
from glob import glob
import os
import random
from typing import Any
from .dataset import RangeDataset, RangeLoader, point_cloud_to_range_image
import numpy as np
import pytorch_lightning as pl
from torch.utils.data import DataLoader, Dataset
from pathlib import Path
import torch
from typing import Optional, Tuple
import h5py
from ouster.sdk import client


class point_cloud_to_range_image_M3ED(point_cloud_to_range_image):
    def __init__(self, 
                **kwargs,) -> None:
        super().__init__(**kwargs)
        # For Ouster sensors, we'll set these dynamically from the first sample
        self.height = None
        self.zenith = None
        self.H = 64  # Ouster typically has 64 beams
        # Flag to indicate we need to initialize calibration
        self.calibration_initialized = False
    
    def initialize_calibration(self, metadata_str):
        """Initialize calibration data from Ouster metadata"""
        metadata = client.SensorInfo(metadata_str)
        # Get beam altitude angles (vertical angles)
        self.zenith = np.array(metadata.beam_altitude_angles, dtype=np.float32)
        # Convert to radians
        self.zenith = np.radians(self.zenith)
        # For Ouster, we don't have separate height values like KITTI
        # We'll use zeros since the Ouster coordinate system is centered at the sensor
        self.height = np.zeros_like(self.zenith)
        self.incl = -self.zenith
        self.calibration_initialized = True
    
    def get_row_inds(self, pc):
        """
        For Ouster, the row indices can be determined directly from the vertical angle
        since the beams have fixed, known positions.
        """
        # Calculate vertical angle for each point
        xy_norm = np.linalg.norm(pc[:, :2], ord=2, axis=1)
        point_vert_angles = np.arctan2(pc[:,2], xy_norm)
        
        # Find the closest beam angle for each point
        error_list = []
        for i in range(len(self.incl)):
            error = np.abs(self.incl[i] - point_vert_angles)
            error_list.append(error)
        all_error = np.stack(error_list, axis=-1)
        row_inds = np.argmin(all_error, axis=-1)
        return row_inds


class M3EDRangeDataset(RangeDataset):
    def __init__(self, 
                 M3ED_path, 
                 train=True, 
                 width=1024, 
                 grid_sizes=[1, 1024, 1024, ], 
                 pc_range=[-25.6, -25.6, -3., 25.6, 25.6, 1.], 
                 log=False,
                 inverse=False,
                 split_ratio=0.9,  # Split ratio for train/test
                 **kwargs):
        super().__init__(**kwargs)
        
        # Find all .h5 files in the M3ED dataset
        h5_files = sorted(glob(os.path.join(M3ED_path, "**/*.h5"), recursive=True))
        
        if len(h5_files) == 0:
            raise RuntimeError(f"No .h5 files found in {M3ED_path}")
        
        # Create a list of all available sweeps across all files
        all_sweeps = []
        
        for h5_path in h5_files:
            with h5py.File(h5_path, 'r') as f:
                # Get the number of sweeps in this file
                num_sweeps = f['/ouster/data'].shape[0]
                print(f"Found {num_sweeps} sweeps in {os.path.basename(h5_path)}")
                
                # Add each (file_path, sweep_index) tuple to our list
                for sweep_idx in range(num_sweeps):
                    all_sweeps.append((h5_path, sweep_idx))
        
        # Apply the train/test split to the list of all sweeps
        total_sweeps = len(all_sweeps)
        split_idx = max(1, min(total_sweeps - 1, int(total_sweeps * split_ratio)))
        
        # Ensure we have at least one sweep in each split
        if total_sweeps > 1:
            if train:
                self.sweeps = all_sweeps[:split_idx]
            else:
                self.sweeps = all_sweeps[split_idx:]
        else:
            # If only one sweep exists, use it for both train and test
            self.sweeps = all_sweeps
            
        print(f"Dataset split: {len(self.sweeps)} sweeps for {'training' if train else 'testing'}")
        
        # Initialize the range image converter
        self.to_range_image = point_cloud_to_range_image_M3ED(
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse
        )
    
    def __len__(self):
        return len(self.sweeps)
    
    def get_pts(self, sweep_info):
        """Extract point cloud from M3ED HDF5 file at specific sweep index"""
        h5_path, sweep_idx = sweep_info
        
        with h5py.File(h5_path, 'r') as f:
            # Get metadata string and initialize calibration if needed
            ouster_metadata_str = f['/ouster/metadata'][...].tolist()
            if not self.to_range_image.calibration_initialized:
                self.to_range_image.initialize_calibration(ouster_metadata_str)
            
            # Get the specified sweep
            metadata = client.SensorInfo(ouster_metadata_str)
            xyzlut = client.XYZLut(metadata)
            
            # Use the specified sweep index
            ouster_sweep = f['/ouster/data'][sweep_idx][...]
            
            # Process the LiDAR packets
            packet_list = []
            for opacket in ouster_sweep:
                packet = client.LidarPacket()
                packet_buf = np.frombuffer(packet.buf, dtype=np.uint8)
                packet_buf[:len(opacket)] = opacket
                packet_list.append(packet)
            
            # Create scans from packets
            packets = client.Packets(packet_list, metadata)
            scans = client.Scans(packets)
            
            # Get the first complete scan
            for scan in scans:
                lidar_scan = scan[0]
                xyz = xyzlut(lidar_scan.field(client.ChanField.RANGE)).reshape(-1, 3)
                signal = lidar_scan.field(client.ChanField.SIGNAL).reshape(-1, 1)
                
                # Combine xyz and signal to match KITTI format (x, y, z, intensity)
                points = np.concatenate([xyz, signal], axis=1).astype(np.float32)
                return points
            
            # If no scan was found, return empty array
            return np.zeros((0, 4), dtype=np.float32)
    
    def get_pth_path(self, sweep_info):
        """Generate a path for caching processed range images that includes sweep index"""
        h5_path, sweep_idx = sweep_info
        cache_dir = os.path.dirname(h5_path).replace('raw', 'range')
        os.makedirs(cache_dir, exist_ok=True)
        # Include sweep index in the filename to avoid collisions
        basename = os.path.basename(h5_path).replace('.h5', f'_sweep{sweep_idx:04d}.pth')
        return os.path.join(cache_dir, basename)
        
    def __getitem__(self, idx):
        sweep_info = self.sweeps[idx]
        pth_path = self.get_pth_path(sweep_info)
        if os.path.exists(pth_path):
            ret = torch.load(pth_path)
        else:
            pts = self.get_pts(sweep_info)
            range_image = self.to_range_image(pts)
            range_image, range_image_mask, car_window_mask = self.to_range_image.process_miss_value(range_image)
            range_image = self.to_range_image.normalize(range_image)
            range_image = torch.from_numpy(range_image).permute(2, 1, 0)  # (C, W, H)
            
            ret = {'jpg': range_image}
            torch.save(ret, pth_path)
            
        return ret


class M3EDRangeLoader(RangeLoader):
    def __init__(self, 
                 M3ED_path, 
                 used_feature=2, 
                 width=1024, 
                 grid_sizes=[1, 1024, 1024, ], 
                 pc_range=[-25.6, -25.6, -3., 25.6, 25.6, 1.], 
                 log=False, 
                 inverse=False, 
                 downsample=None,
                 inpainting=None,
                 coord=False,
                 **kwargs):
        super().__init__(**kwargs)
        
        train_dataset = M3EDRangeDataset(
            M3ED_path, 
            train=True, 
            used_feature=used_feature, 
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse,
            downsample=downsample,
            inpainting=inpainting,
            coord=coord
        )
        
        test_dataset = M3EDRangeDataset(
            M3ED_path, 
            train=False, 
            used_feature=used_feature, 
            width=width, 
            grid_sizes=grid_sizes, 
            pc_range=pc_range, 
            log=log,
            inverse=inverse,
            downsample=downsample,
            inpainting=inpainting,
            coord=coord
        )
        
        self.train_dataset = train_dataset
        self.test_dataset = test_dataset


if __name__ == "__main__":
    import argparse
    from PIL import Image
    import numpy as np

    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, help="M3ED dataset root folder")
    parser.add_argument("--save", default=None, help="optional PNG output path")
    args = parser.parse_args()

    # Build a tiny loader (one sample, CPU)
    loader = M3EDRangeLoader(
        M3ED_path=args.root,
        batch_size=1,
        num_workers=0,
        shuffle=False,
    )
    
    sample = next(iter(loader.test_dataloader()))
    rng = sample["jpg"][0]          # (C, W, H)  –- here C==2

    print("Range-image tensor shape:", rng.shape)       # e.g. torch.Size([2,1024,64])

    if args.save:
        # Take only channel-0 (range), un-normalize, map to 0-255 for display
        ch0 = rng[0] * loader.test_dataset.to_range_image.std \
                       + loader.test_dataset.to_range_image.mean
        ch0 = ch0.cpu().numpy() 
        img = (ch0/ ch0.max() * 255).astype("uint8").T  # H×W
        Image.fromarray(img).save(args.save)
        print("Saved PNG to", args.save)

        # Save intensity image
        ch1 = rng[1].cpu().numpy()
        ch1_img = (ch1 / ch1.max() * 255).astype(np.uint8).T
        inten_path = args.save.replace(".png", "_intensity.png")
        Image.fromarray(ch1_img).save(inten_path)
        print("Saved intensity PNG to", inten_path)