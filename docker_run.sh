#!/bin/bash
# docker_run.sh: Script to launch the real hardware container for Prisma Rover

FORCE_SOFTWARE=false
CONTAINER_NAME="prisma_rover_real_container"
IMAGE_NAME="prisma_rover:real"

for arg in "$@"; do
    if [ "$arg" = "software" ] || [ "$arg" = "cpu" ] || [ "$arg" = "--software" ]; then
        FORCE_SOFTWARE=true
    else
        CONTAINER_NAME="$arg"
    fi
done

echo "=== Starting Real Hardware Docker Container ==="
echo "Image Name:       $IMAGE_NAME"
echo "Container Name:   $CONTAINER_NAME"
echo "================================================"

# Allow local GUI connections to X11 (for RViz inside container)
xhost +local:root &>/dev/null || true

# Automatic GPU detection for hardware rendering
GPU_FLAGS=""
ENV_FLAGS=""

if [ "$FORCE_SOFTWARE" = "true" ]; then
    echo "[GPU] Mesa Software Rendering forced (llvmpipe)."
    ENV_FLAGS="--env=LIBGL_ALWAYS_SOFTWARE=true --env=MESA_GL_VERSION_OVERRIDE=3.3"
elif command -v nvidia-smi &> /dev/null && docker info 2>&1 | grep -iq "nvidia"; then
    echo "[GPU] NVIDIA GPU detected. Enabling NVIDIA hardware acceleration."
    GPU_FLAGS="--gpus all"
elif [ -d "/dev/dri" ]; then
    echo "[GPU] Intel/AMD DRI graphics card detected. Enabling hardware acceleration."
    GPU_FLAGS="--device /dev/dri:/dev/dri"
    
    # Add video and render GIDs to prevent Permission Denied
    SETUP_CMDS=""
    if getent group video &>/dev/null; then
        VIDEO_GID=$(getent group video | cut -d: -f3)
        GPU_FLAGS="$GPU_FLAGS --group-add $VIDEO_GID"
        SETUP_CMDS="$SETUP_CMDS sudo groupadd -g $VIDEO_GID host_video &>/dev/null; sudo usermod -aG host_video user &>/dev/null;"
    fi
    if getent group render &>/dev/null; then
        RENDER_GID=$(getent group render | cut -d: -f3)
        GPU_FLAGS="$GPU_FLAGS --group-add $RENDER_GID"
        SETUP_CMDS="$SETUP_CMDS sudo groupadd -g $RENDER_GID host_render &>/dev/null; sudo usermod -aG host_render user &>/dev/null;"
    fi
else
    echo "[GPU] No GPU detected. Enabling Mesa software rendering (llvmpipe)."
    ENV_FLAGS="--env=LIBGL_ALWAYS_SOFTWARE=true --env=MESA_GL_VERSION_OVERRIDE=3.3"
fi

# Default ROS_LOCALHOST_ONLY to 1 if not defined (keeps network traffic local by default)
ROS_LH="${ROS_LOCALHOST_ONLY:-1}"

# Start container with host networking, privileged permissions, and USB/Serial/Dev mounts
if [ -n "$SETUP_CMDS" ]; then
    docker run --rm -it \
      --name="$CONTAINER_NAME" \
      --net=host \
      --ipc=host \
      --privileged \
      $GPU_FLAGS \
      $ENV_FLAGS \
      --env="DISPLAY=$DISPLAY" \
      --env="ROS_LOCALHOST_ONLY=$ROS_LH" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
      --volume="/dev:/dev" \
      --volume="$(pwd)/ros2_ws:/home/user/ros2_ws" \
      "$IMAGE_NAME" \
      bash -c "$SETUP_CMDS exec bash"
else
    docker run --rm -it \
      --name="$CONTAINER_NAME" \
      --net=host \
      --ipc=host \
      --privileged \
      $GPU_FLAGS \
      $ENV_FLAGS \
      --env="DISPLAY=$DISPLAY" \
      --env="ROS_LOCALHOST_ONLY=$ROS_LH" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
      --volume="/dev:/dev" \
      --volume="$(pwd)/ros2_ws:/home/user/ros2_ws" \
      "$IMAGE_NAME" \
      bash -c "exec bash"
fi

# Restore X11 permissions on exit
xhost -local:root &>/dev/null || true