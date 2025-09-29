#!/bin/bash
set -e

# Source ROS and the workspace
source "/opt/ros/noetic/setup.bash"
# Use the correct path for the developer user's workspace
if [ -f "/home/developer/dlio_ws/devel/setup.bash" ]; then
    source "/home/developer/dlio_ws/devel/setup.bash"
fi

# Execute the command passed to the docker run command
exec "$@"