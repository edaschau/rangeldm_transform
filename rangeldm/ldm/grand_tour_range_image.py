"""
Grand Tour dataset loader for LDM training.
Wraps the VAE data loader for use with the diffusion model.
"""
import os
import sys
import importlib.util

# Add the vae/sgm/data path to import the Grand Tour loader
vae_data_path = os.path.join(os.path.dirname(__file__), '../vae/sgm/data')
vae_data_path = os.path.abspath(vae_data_path)

# First, ensure the parent packages are properly set up
# We need to add the sgm package path and set up the package structure
vae_sgm_path = os.path.join(os.path.dirname(__file__), '../vae/sgm')
vae_sgm_path = os.path.abspath(vae_sgm_path)

# Add to sys.path if not already there
if vae_sgm_path not in sys.path:
    sys.path.insert(0, vae_sgm_path)
if vae_data_path not in sys.path:
    sys.path.insert(0, vae_data_path)

# Import the base classes directly (these don't have circular import issues)
from data.dataset import RangeDataset, RangeLoader, point_cloud_to_range_image

# Now import the Grand Tour module from the data package
# This works because we've added the sgm path and can use package imports
from data import grand_tour_range_image as vae_grand_tour_module

import numpy as np
import torch

# Get the VAE's GrandTourRangeDataset and related classes
VAEGrandTourDataset = vae_grand_tour_module.GrandTourRangeDataset
point_cloud_to_range_image_GrandTour = vae_grand_tour_module.point_cloud_to_range_image_GrandTour


class GrandTourRangeDataset(RangeDataset):
    """
    Wrapper around the VAE Grand Tour dataset for LDM training.
    """
    def __init__(self,
                 grand_tour_path,
                 train=True,
                 width=1024,
                 grid_sizes=[1, 1024, 1024],
                 pc_range=[-25.6, -25.6, -3., 25.6, 25.6, 1.],
                 log=False,
                 inverse=False,
                 topic_name='/boxi/hesai/points',
                 num_beams=32,
                 **kwargs):
        super().__init__(**kwargs)
        
        # Create the VAE dataset (handles ROS2 bag reading)
        self.vae_dataset = VAEGrandTourDataset(
            grand_tour_path=grand_tour_path,
            train=train,
            width=width,
            grid_sizes=grid_sizes,
            pc_range=pc_range,
            log=log,
            inverse=inverse,
            topic_name=topic_name,
            num_beams=num_beams,
            # Don't pass inpainting/downsample to VAE dataset, we handle it here
        )
        
        self.to_range_image = self.vae_dataset.to_range_image
        self.file_paths = self.vae_dataset.messages  # For compatibility
    
    def __len__(self):
        return len(self.vae_dataset)
    
    def get_pts(self, message_info):
        return self.vae_dataset.get_pts(message_info)
    
    def get_pth_path(self, message_info):
        return self.vae_dataset.get_pth_path(message_info)
    
    def __getitem__(self, idx):
        # Get the base item from VAE dataset (only contains 'jpg')
        ret = self.vae_dataset[idx]
        
        # Apply downsample if configured
        if self.downsample:
            if isinstance(self.downsample, int):
                self.downsample = [1, self.downsample]
            range_image = ret['jpg']
            ret['down'] = range_image[:, 
                                      (self.downsample[0]//2)::self.downsample[0], 
                                      (self.downsample[1]//2)::self.downsample[1]]
        
        # Apply inpainting mask if configured
        if self.inpainting:
            range_image = ret['jpg']
            C, W, H = range_image.shape
            inpainting_mask = -torch.ones(1, W, H)
            inpainting_start = 0.0
            inpainting_end = inpainting_start + self.inpainting
            if inpainting_end < 1.0:
                inpainting_mask[:, int(inpainting_start*W):int(inpainting_end*W), :] = 1
            else:
                inpainting_mask[:, int(inpainting_start*W):, :] = 1
                inpainting_mask[:, :int((inpainting_end-1.0)*W), :] = 1
            masked_image = -torch.ones_like(range_image)
            masked_image[:, inpainting_mask.squeeze(0) < 0] = range_image[:, inpainting_mask.squeeze(0) < 0]
            ret['inpainting_mask'] = inpainting_mask
            ret['masked_image'] = masked_image
        
        return ret


class GrandTourRangeLoader(RangeLoader):
    """
    DataLoader for Grand Tour dataset for LDM training.
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
                 topic_name='/boxi/hesai/points',
                 num_beams=32,
                 **kwargs):
        super().__init__(**kwargs)
        
        self.train_dataset = GrandTourRangeDataset(
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
        
        self.test_dataset = GrandTourRangeDataset(
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
