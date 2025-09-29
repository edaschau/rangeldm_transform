# Run dlio with Anymal GrandTour dataset

## Build the whole container
See `Dockerfile` in `dlio_project` directory. Created and tested on Ubuntu 24.04

```bash
sudo docker build -t dlio .
```
## First, ensure GUI forwarding is enabled

```bash
xhost +local:docker
```
## Run the container

```bash
sudo docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)/dlio_data:/home/developer/data:ro" \
  dlio
```

## Then in 3 separate docker terminal sessions, run the following commands:
```bash
# initialize roscore
roscore
```

```bash
# launch dlio 
roslaunch direct_lidar_inertial_odometry dlio.launch
```

```bash
# play all the bags contains data
# replace the bag name according to your download
rosbag play --clock /home/developer/data/2024-11-02-17-10-25_hesai_undist.bag /home/developer/data/2024-11-02-17-10-25_stim320_imu.bag /home/developer/data/2024-11-02-17-10-25_tf_model.bag
```