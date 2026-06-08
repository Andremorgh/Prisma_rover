#!/bin/bash
# docker_run.sh: Script to launch the single development container for Prisma Rover

# Parse options
FORCE_SOFTWARE=false
PROFILE="base"
CONTAINER_NAME="prisma_rover_container"

for arg in "$@"; do
    if [ "$arg" = "software" ] || [ "$arg" = "cpu" ] || [ "$arg" = "--software" ]; then
        FORCE_SOFTWARE=true
    elif [ "$arg" = "base" ] || [ "$arg" = "yolo" ] || [ "$arg" = "quantum" ] || [ "$arg" = "full" ]; then
        PROFILE="$arg"
    else
        CONTAINER_NAME="$arg"
    fi
done

IMAGE_NAME="prisma_rover:$PROFILE"

echo "=== Starting Docker Container ==="
echo "Selected Profile: $PROFILE"
echo "Image Name:       $IMAGE_NAME"
echo "Container Name:   $CONTAINER_NAME"
echo "=================================="

# Allow local GUI connections to X11
xhost +local:root

# Automatic GPU detection
GPU_FLAGS=""
ENV_FLAGS=""

if [ "$FORCE_SOFTWARE" = "true" ]; then
    echo "[GPU] Mesa Software Rendering forced (llvmpipe)."
    ENV_FLAGS="--env=LIBGL_ALWAYS_SOFTWARE=true --env=MESA_GL_VERSION_OVERRIDE=3.3"
elif command -v nvidia-smi &> /dev/null && docker info 2>&1 | grep -iq "nvidia"; then
    echo "[GPU] NVIDIA GPU detected with configured runtime. Enabling NVIDIA hardware acceleration."
    GPU_FLAGS="--gpus all"
elif [ -d "/dev/dri" ]; then
    echo "[GPU] DRI device (/dev/dri) detected for AMD/Intel GPU. Enabling open-source hardware acceleration."
    GPU_FLAGS="--device /dev/dri:/dev/dri"
    
    # Add host video and render GIDs to prevent Permission Denied inside the container
    # and dynamically register them inside the container to prevent shell warning
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
    echo "[GPU] No supported GPU detected. Enabling Mesa software rendering (llvmpipe)."
    ENV_FLAGS="--env=LIBGL_ALWAYS_SOFTWARE=true --env=MESA_GL_VERSION_OVERRIDE=3.3"
fi

# Start the container in interactive mode with hardware permissions and workspace mount
if [ -n "$SETUP_CMDS" ]; then
    docker run --rm -it \
      --name="$CONTAINER_NAME" \
      --net=host \
      --ipc=host \
      --privileged \
      $GPU_FLAGS \
      $ENV_FLAGS \
      --env="DISPLAY=$DISPLAY" \
      --env="ROS_LOCALHOST_ONLY=1" \
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
      --env="ROS_LOCALHOST_ONLY=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
      --volume="/dev:/dev" \
      --volume="$(pwd)/ros2_ws:/home/user/ros2_ws" \
      "$IMAGE_NAME" \
      bash -c "exec bash"
fi

# Restore X11 permissions on exit
xhost -local:root