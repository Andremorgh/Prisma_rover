#!/bin/bash
set -e

# Source ROS 2 base installation
source "/opt/ros/humble/setup.bash"

# Source CycloneDDS settings if RMW is set
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "/home/user/cyclonedds_config.xml" ]; then
    export CYCLONEDDS_URI="file:///home/user/cyclonedds_config.xml"
fi

# Source workspace install if it exists
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
    source "/home/user/ros2_ws/install/setup.bash"
fi

echo "[Entrypoint] Prisma Rover Simulation Workspace Ready"
echo "  - ROS 2 Humble sourced"
echo "  - CycloneDDS active"
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
    echo "  - Workspace overlay sourced"
fi

# Execute the command passed to the container
exec "$@"
