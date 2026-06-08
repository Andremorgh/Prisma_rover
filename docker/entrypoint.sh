#!/bin/bash
set -e

# Source ROS 2 base installation
source "/opt/ros/humble/setup.bash"

# Source CycloneDDS settings if RMW is set
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "/home/user/cyclonedds_config.xml" ]; then
    export CYCLONEDDS_URI="file:///home/user/cyclonedds_config.xml"
fi

# Define modular packages lists
YOLO_PKGS=("yolov11_ros2" "obj_detection" "prisma_rover_obj_det_agent" "prisma_rover_perception" "prisma_rover_explorer")
QUANTUM_PKGS=("prisma_rover_quantum_controller")
ALL_MODULAR_PKGS=("${YOLO_PKGS[@]}" "${QUANTUM_PKGS[@]}")

# Setup workspace packages mapping based on profile
if [ -d "/home/user/ros2_ws/src" ]; then
    # First, remove COLCON_IGNORE from all modular packages to reset state
    for pkg in "${ALL_MODULAR_PKGS[@]}"; do
        if [ -f "/home/user/ros2_ws/src/$pkg/COLCON_IGNORE" ]; then
            rm "/home/user/ros2_ws/src/$pkg/COLCON_IGNORE"
        fi
    done

    # Now, add COLCON_IGNORE to excluded packages based on active profile
    case "$ROVER_PROFILE" in
        "base")
            echo "[Entrypoint] Active Profile: base. This profile includes:"
            echo "  - Ignition Gazebo Simulation"
            echo "  - Nav2 Navigation Stack"
            echo "  - SLAM Toolbox / RTAB-Map"
            echo "  - Robot Localization (EKF)"
            echo "  - Robot Description (URDF/Xacro)"
            echo "  - Teleoperation"
            for pkg in "${ALL_MODULAR_PKGS[@]}"; do
                if [ -d "/home/user/ros2_ws/src/$pkg" ]; then
                    touch "/home/user/ros2_ws/src/$pkg/COLCON_IGNORE"
                fi
            done
            ;;
        "yolo")
            echo "[Entrypoint] Active Profile: yolo. This profile includes:"
            echo "  - Ignition Gazebo Simulation"
            echo "  - Nav2 Navigation Stack"
            echo "  - SLAM Toolbox / RTAB-Map"
            echo "  - Robot Localization (EKF)"
            echo "  - Robot Description (URDF/Xacro)"
            echo "  - Teleoperation"
            echo "  - YOLOv11 Target Tracking & Semantic Mapping"
            echo "  - Reinforcement Learning (RL) Search Agent"
            echo "  - ArUco Marker Detection & Pose Estimation"
            for pkg in "${QUANTUM_PKGS[@]}"; do
                if [ -d "/home/user/ros2_ws/src/$pkg" ]; then
                    touch "/home/user/ros2_ws/src/$pkg/COLCON_IGNORE"
                fi
            done
            ;;
        "quantum")
            echo "[Entrypoint] Active Profile: quantum. This profile includes:"
            echo "  - Ignition Gazebo Simulation"
            echo "  - Nav2 Navigation Stack"
            echo "  - SLAM Toolbox / RTAB-Map"
            echo "  - Robot Localization (EKF)"
            echo "  - Robot Description (URDF/Xacro)"
            echo "  - Teleoperation"
            echo "  - Reactive Fuzzy Obstacle Avoidance Controller (LUT)"
            for pkg in "${YOLO_PKGS[@]}"; do
                if [ -d "/home/user/ros2_ws/src/$pkg" ]; then
                    touch "/home/user/ros2_ws/src/$pkg/COLCON_IGNORE"
                fi
            done
            ;;
        "full")
            echo "[Entrypoint] Active Profile: full. This profile includes:"
            echo "  - Ignition Gazebo Simulation"
            echo "  - Nav2 Navigation Stack"
            echo "  - SLAM Toolbox / RTAB-Map"
            echo "  - Robot Localization (EKF)"
            echo "  - Robot Description (URDF/Xacro)"
            echo "  - Teleoperation"
            echo "  - YOLOv11 Target Tracking & Semantic Mapping"
            echo "  - Reinforcement Learning (RL) Search Agent"
            echo "  - ArUco Marker Detection & Pose Estimation"
            echo "  - Reactive Fuzzy Obstacle Avoidance Controller (LUT)"
            ;;
        *)
            echo "[Entrypoint] Warning: ROVER_PROFILE is not set or unknown ($ROVER_PROFILE). Leaving workspace unmodified."
            ;;
    esac
fi

# Source workspace install if it exists
if [ -f "/home/user/ros2_ws/install/setup.bash" ]; then
    source "/home/user/ros2_ws/install/setup.bash"
fi

# Execute the command passed to the container
exec "$@"
