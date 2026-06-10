#!/bin/bash
set -e

# Source ROS 2 base installation
source "/opt/ros/humble/setup.bash"

# Source CycloneDDS settings
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "/home/user/cyclonedds_config.xml" ]; then
    export CYCLONEDDS_URI="file:///home/user/cyclonedds_config.xml"
fi

# Source global Livox driver setup if it exists
if [ -f "/opt/livox_ws/install/setup.bash" ]; then
    source "/opt/livox_ws/install/setup.bash"
fi

# Clean up any residual COLCON_IGNORE files in the workspace to ensure all packages build
if [ -d "/home/user/ros2_ws/src" ]; then
    echo "[Entrypoint] Cleaning up residual COLCON_IGNORE files in workspace..."
    find /home/user/ros2_ws/src -name "COLCON_IGNORE" -delete || true
fi

# Source workspace install overlay if it exists
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
    source "/home/user/ros2_ws/install/setup.bash"
fi

echo "[Entrypoint] Physical Rover Hardware Environment Initialized."
echo "Active packages list:"
ros2 pkg list | grep prisma_rover || true

# Execute the command passed to the container
exec "$@"
