# Stage 1: Base image with ROS and dependencies
FROM osrf/ros:noetic-desktop-full AS base

# Install dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:oibaf/graphics-drivers \
    && apt-get update && apt-get install -y \
    mesa-utils \
    x11-apps \
    xvfb \
    qt5-default \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    nano \
    git \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gmapping \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-octomap \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-mapping \
    ros-noetic-octomap-server \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-octomap-msgs \
    ros-noetic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for Qt and display
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Stage 2: Clone repository
FROM base AS clone
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN echo "Cloning NavRover repository..." && \
    git clone https://github.com/jj7258/NavRover.git --progress && \
    echo "Repository cloned successfully."

# Stage 3: Build workspace
FROM clone AS build
WORKDIR /root/catkin_ws
RUN echo "Building catkin workspace..." && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -j2" && \
    echo "Catkin workspace built successfully."

# Stage 4: Final image
FROM build AS final

# Setup environment
RUN echo "Setting up ROS environment..." && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "ROS environment setup complete."

# Create XDG_RUNTIME_DIR for GUI applications
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Create and set up the entrypoint script
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
# Final message
RUN echo "NavRover Docker image build complete!"
