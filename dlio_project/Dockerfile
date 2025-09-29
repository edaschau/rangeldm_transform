FROM ros:noetic-robot-focal

# Make bash the default shell
SHELL ["/bin/bash", "-c"]

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools only
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basic tools
    git wget nano curl build-essential python3-pip python3-catkin-tools sudo \
    # PPA tool
    software-properties-common \
    # DLIO Dependencies
    libpcl-dev \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user (optional)
RUN useradd -m developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer

# # Create a catkin workspace
# WORKDIR /home/developer/dlio_ws
# RUN mkdir -p src && chown -R developer:developer /home/developer/dlio_ws
# RUN catkin build
# RUN source ~/dlio_ws/devel/setup.bash

# Create a catkin workspace and set it as the working directory
WORKDIR /home/developer/dlio_ws
RUN mkdir src && chown -R developer:developer /home/developer/dlio_ws

# Switch to the developer user to ensure correct file ownership
USER developer

# 1. COPY the source code into the workspace FIRST
COPY --chown=developer:developer dlio ./src/dlio

# 2. RUN catkin build, sourcing the ROS environment
RUN source /opt/ros/noetic/setup.bash && catkin build

# Copy the entrypoint script
USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


# RUN source ~/dlio_ws/devel/setup.bash

USER developer
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]