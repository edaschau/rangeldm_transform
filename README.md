# RangeLDM Transform

LiDAR range image generation and inpainting using RangeLDM (VAE + Latent Diffusion Model). Includes support for KITTI-360 and the ANYmal Grand Tour dataset (Hesai XT-32, 32 beams).

## Repository Structure

- `rangeldm/vae/` — Variational Autoencoder (PyTorch Lightning / sgm framework)
- `rangeldm/ldm/` — Latent Diffusion Model (HuggingFace diffusers / accelerate)
- `utils/to_rosbag2.py` — ROS1 bag → ROS2 bag conversion
- `dlio_project/` — DLIO demo with Hesai LiDAR from the ANYmal Grand Tour dataset
- `tests_src/` — Utility and visualization scripts

## ROS1 → ROS2 Bag Conversion

Convert Grand Tour ROS1 bags to ROS2 format (required before training):

```bash
# Edit utils/to_rosbag2.py to set input/output paths, then:
python utils/to_rosbag2.py
```

The script uses `rosbags.convert.converter`. Set `ros1_bag` to your `.bag` file and `ros2_bag` to the desired output directory. Example:

```python
ros1_bag = Path('grand_tour_dataset/data_ros1/2024-11-02-17-10-25_hesai.bag')
ros2_bag = Path('grand_tour_dataset/data_ros2/anymal_hesai_ros2')
```

Requires: `pip install rosbags`

## Training: Inpainting with Grand Tour Dataset

### Step 1 — Train the VAE

```bash
cd rangeldm/vae
python main.py \
  -b configs/grand_tour.yaml \
  -l /path/to/vae_output \
  "data.params.grand_tour_path=/absolute/path/to/grand_tour_dataset/data_ros2/anymal_hesai_ros2"
```

- `-b` — base config (encoder/decoder architecture, loss weights, learning rate)
- `-l` — log directory for checkpoints and metrics (outside this repo)
- The `data.params.grand_tour_path` override sets the absolute path to the ROS2 bag directory

Key config parameters (`configs/grand_tour.yaml`):
- `base_learning_rate: 1.0e-4`, `disc_start: 5000`, `max_epochs: 500`
- 2-channel input (range + intensity), 1024×32 resolution, latent dim 4

### Step 2 — Train the LDM (Inpainting)

Create a runtime YAML config with absolute paths (or edit `configs/grand_tour_inpainting.yaml` directly):

```yaml
vae_config: /absolute/path/to/rangeldm/vae/configs/grand_tour.yaml
vae_checkpoint: /absolute/path/to/vae_output/<run_name>/checkpoints/epoch=XXXXXX.ckpt
output_dir: /path/to/ldm_output
grand_tour_path: /absolute/path/to/grand_tour_dataset/data_ros2/anymal_hesai_ros2
```

Then run:

```bash
cd rangeldm/ldm
python train_conditional.py --cfg /path/to/your_config.yaml
```

The UNet is auto-configured: 9 input channels (4 noisy latent + 4 masked latent + 1 inpainting mask), 4 output channels. Checkpoints are saved every 500 steps to `output_dir/`.

## Other Datasets

- **KITTI-360**: Set `KITTI360_DATASET` env var; use `configs/inpainting.yaml`
- **nuScenes**: Use `configs/nuscenes.yaml`

## Notes

- `dlio_project/` — Complete DLIO demo with Hesai LiDAR from the ANYmal Grand Tour dataset
- `mola/` — MOLA SLAM config (experimental)
- `tests_src/` — Bag inspection, TF analysis, visualization launch files

Contact: zydwork@outlook.com