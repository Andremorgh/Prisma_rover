# ==========================================
# Real Robot Hardware ROS 2 Environment
# ==========================================
FROM ros:humble

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install base dev tools, colcon, and CycloneDDS
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    dialog \
    apt-utils \
    apt-transport-https \
    curl \
    lsb-release \
    gnupg \
    kmod \
    minicom \
    screen \
    tmux \
    tmuxp \
    git \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install hardware drivers and navigation/SLAM stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-realsense2-camera \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-rtabmap-ros \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-rplidar-ros \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies for ArUco and motor drivers
RUN pip3 install --no-cache-dir \
    numpy \
    transforms3d \
    opencv-python \
    pyserial

# Compile and install Livox SDK2 and livox_ros_driver2
# (Required for Ethernet UDP LiDAR communication)
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Livox-SDK2

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /tmp/livox_ws/src/livox_ros_driver2 && \
    cd /tmp/livox_ws/src/livox_ros_driver2 && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && ./build.sh ROS2" && \
    mkdir -p /opt/livox_ws && \
    mv /tmp/livox_ws/install /opt/livox_ws/install && \
    rm -rf /tmp/livox_ws

# Add non-root user using UID and GID passed as build arguments
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create group and user with matching host IDs to prevent file permission issues
RUN groupadd --gid $GROUP_ID user || true && \
    useradd --uid $USER_ID --gid $GROUP_ID --create-home --shell /bin/bash user && \
    echo "user:user" | chpasswd && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG dialout user && \
    usermod -aG video user && \
    usermod -aG plugdev user || true

# Create workspace directories
ENV HOME=/home/user
ENV WORKSPACE_DIR=$HOME/ros2_ws
RUN mkdir -p $WORKSPACE_DIR/src && chown -R user:user $HOME

USER user
WORKDIR $WORKSPACE_DIR

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=90

# Copy CycloneDDS configuration
COPY --chown=user docker/cyclonedds_config.xml $HOME/cyclonedds_config.xml
ENV CYCLONEDDS_URI=file://$HOME/cyclonedds_config.xml

# Set up entrypoint
COPY --chown=user docker/entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
USER user

# Configure automatically sourcing setups inside interactive shells
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /opt/livox_ws/install/setup.bash ]; then source /opt/livox_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
