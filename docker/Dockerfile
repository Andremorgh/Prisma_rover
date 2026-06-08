# ==========================================
# STAGE 1: Base ROS 2 Humble Environment
# ==========================================
FROM ros:humble AS base-deps

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

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


# ==========================================
# STAGE 2: Simulation & Nav2 Dependencies
# ==========================================
FROM base-deps AS sim-deps

USER root

# Install Gazebo/Ignition Fortress and ROS-Gazebo bridges
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

USER user


# ==========================================
# STAGE 3: YOLO / Perception Dependencies
# ==========================================
FROM sim-deps AS yolo-deps

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-tf-transformations \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy && \
    pip3 install --no-cache-dir \
    ultralytics \
    lap \
    lapx \
    open3d \
    shapely \
    scikit-learn \
    ros2-numpy

USER user


# ==========================================
# STAGE 4: Quantum Dependencies
# ==========================================
FROM sim-deps AS quantum-deps

USER root
RUN pip3 install --no-cache-dir scipy
USER user


# ==========================================
# STAGE 5: Combined/Full Dependencies
# ==========================================
FROM yolo-deps AS full-deps

USER root
RUN pip3 install --no-cache-dir scipy
USER user


# ==========================================
# TARGETS: Compilation / Run Profiles
# ==========================================

# PROFILE: base
FROM sim-deps AS base
ENV ROVER_PROFILE=base

USER root
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc
USER user


# PROFILE: yolo
FROM yolo-deps AS yolo
ENV ROVER_PROFILE=yolo

USER root
RUN usermod -aG dialout user && \
    usermod -aG video user && \
    usermod -aG plugdev user || true
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc
USER user


# PROFILE: quantum
FROM quantum-deps AS quantum
ENV ROVER_PROFILE=quantum

USER root
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc
USER user


# PROFILE: full
FROM full-deps AS full
ENV ROVER_PROFILE=full

USER root
RUN usermod -aG dialout user && \
    usermod -aG video user && \
    usermod -aG plugdev user || true
RUN echo "source /opt/ros/humble/setup.bash;" >> /home/user/.bashrc && \
    echo "if [ -f /home/user/ros2_ws/install/setup.bash ]; then source /home/user/ros2_ws/install/setup.bash; fi" >> /home/user/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}" >> /home/user/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///home/user/cyclonedds_config.xml" >> /home/user/.bashrc
USER user
