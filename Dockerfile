FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    ros-noetic-move-base-flex \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /catkin_ws
RUN mkdir -p src

# Copy package
COPY . src/path_tracking_pid/

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

CMD ["/bin/bash"]