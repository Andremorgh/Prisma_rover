# ==========================================
# Single-Stage ROS 2 Humble Simulation Environment
# ==========================================
FROM ros:humble

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install base dev dependencies and CycloneDDS
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
    python3-setuptools \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Add non-root user using UID and GID passed as build arguments
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create group and user with matching host IDs to prevent file permission issues
RUN groupadd --gid $GROUP_ID user || true && \
    useradd --uid $USER_ID --gid $GROUP_ID --create-home --shell /bin/bash user && \
    echo "user:user" | chpasswd && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create workspace directories
ENV HOME=/home/user
ENV WORKSPACE_DIR=$HOME/ros2_ws
RUN mkdir -p $WORKSPACE_DIR/src && chown -R user:user $HOME

WORKDIR $WORKSPACE_DIR

# Install Gazebo/Ignition Fortress, ROS-Gazebo bridges, Nav2, and SLAM dependencies
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    assimp-utils \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gz \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-libg2o \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# Install ArUco Marker Detection & Camera bridge dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Install python packages required by ArUco pose estimation and utils
RUN pip3 install --no-cache-dir \
    numpy \
    transforms3d \
    ros2-numpy

# Set up user permissions for hardware acceleration DRI/Video GIDs (AMD/Intel/NVIDIA)
RUN usermod -aG dialout user && \
    usermod -aG video user && \
    usermod -aG plugdev user || true

USER user

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=90

# Copy CycloneDDS configuration
COPY --chown=user docker/cyclonedds_config.xml $HOME/cyclonedds_config.xml
ENV CYCLONEDDS_URI=file://$HOME/cyclonedds_config.xml

# Set up automatic environment sourcing in bashrc
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc

# Set up entrypoint
COPY --chown=user docker/entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
USER user

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
