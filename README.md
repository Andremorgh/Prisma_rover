# Prisma Rover: Autonomous Simulation Workspace

An advanced, modular ROS 2 Humble robotic workspace designed for autonomous 3D navigation, SLAM mapping, frontier-based exploration, and ArUco marker perception.

![Prisma Rover](docs/real_rover.png)

> [!NOTE]
> For more detailed technical specifications, changes log, and design notes, please refer to the [docs/](file:///home/andrea/Desktop/Prisma_rover/docs) folder.

---

## Table of Contents
1. [Workspace Architecture](#workspace-architecture)
2. [Docker Integration & Build Setup](#docker-integration--build-setup)
3. [Installation & Build](#installation--build)
4. [Running the Simulation](#running-the-simulation)
5. [Simulation FAQ](#simulation-faq)

---

## Workspace Architecture

The workspace is organized into modular ROS 2 packages under `ros2_ws/src/`, categorized by functionality:

### 1. Core Navigation & Description
* **[prisma_rover_description](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_description)**: Contains the URDF model, physics parameters, and 3D Visual CAD meshes representing the rover chassis and wheels.
* **[prisma_rover_sim](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_sim)**: Houses the Gazebo Ignition worlds (maze, warehouse, depot, obstacle_test, object_world, etc.), and configures topic bridges via `ros_gz_bridge`.
* **[prisma_rover_localization](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_localization)**: Configures the Extended Kalman Filter (EKF) state estimation nodes, fusing wheel odometry and IMU data.
* **[prisma_rover_navigation](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_navigation)**: Coordinates SLAM Toolbox configurations and Nav2 costmap/path-planner parameters.

### 2. Control & Exploration
* **[prisma_rover_explorer](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_explorer)**: Handles frontier-based coverage algorithms for autonomous exploration and sweep coverage.
* **[prisma_rover_manager](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_manager)**: High-level C++ orchestrator coordinating the transitions between localization, mapping, and exploration states.
* **[prisma_rover_teleop](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_teleop)**: Keyboard and joystick teleoperation profiles.

### 3. Perception & Custom Interfaces
* **[aruco_detector](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_perception/aruco_detector)**: Identifies ArUco markers within video frames and broadcasts TF transforms for each detected marker.
* **[aruco_pose_estimation](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_perception/aruco_pose_estimation)**: Synchronizes RGB and Depth camera streams to estimate and log the exact 3D coordinates of detected markers.
* **[prisma_rover_interfaces](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_interfaces)**: Defines custom ROS 2 message structures (`ArucoMarkers`).

---

## Docker Integration & Build Setup

This workspace utilizes a simplified, single-stage Docker container environment that installs all dependencies required for the simulation, navigation stack, and ArUco perception. 

The environment builds as a single target image (`prisma_rover:simulation`) that compiles the entire workspace in one go.

---

## Installation & Build

### Prerequisites
* Docker installed on your host system
* NVIDIA Container Toolkit (Optional, for GPU acceleration)

### Steps
1. Build the Docker simulation image:
   ```bash
   ./docker_build.sh
   ```
2. Launch the container:
   ```bash
   ./docker_run.sh
   ```
3. Inside the container, compile the ROS 2 workspace:
   ```bash
   colcon build --symlink-install
   ```

---

## Running the Simulation

To launch the full Gazebo simulation along with robot localization, SLAM mapping, and Nav2 navigation:

```bash
ros2 launch prisma_rover_navigation sim_navigation.launch.py rviz:=true
```

To run navigation with ArUco marker pose detection enabled:
```bash
ros2 launch prisma_rover_navigation aruco_navigation.launch.py rviz:=true
```

To run keyboard teleoperation (in another container terminal):
```bash
ros2 run prisma_rover_teleop teleop_keyboard --ros-args -r __ns:=/prisma_rover
```

---

## Simulation FAQ

### Q1: How can I run the Gazebo simulation in headless mode (no GUI)?
To run the simulation without launching the Gazebo GUI (useful for background runs or low-resource hosts), set the `headless:=true` parameter. For example:
```bash
ros2 launch prisma_rover_navigation sim_navigation.launch.py headless:=true
```

### Q2: How do I prevent ROS 2 from saturating my local network?
By default, ROS 2 uses multicast for node discovery, which can flood your physical network. To prevent this, the workspace comes configured with **Localhost Isolation** enabled inside Docker:
1. `docker_run.sh` sets `--env="ROS_LOCALHOST_ONLY=1"`. This confines DDS traffic strictly to the localhost loopback interface.
2. If you want to communicate with external ROS 2 nodes on your network, you can set `ROS_LOCALHOST_ONLY=0` in your run script, but it is highly recommended to specify a unique `ROS_DOMAIN_ID` (e.g. `export ROS_DOMAIN_ID=42`) to avoid crosstalk.

### Q3: Can I exclude the camera from the simulation to reduce rendering overhead?
Yes. You can disable the camera sensors and camera-to-ROS bridge by setting the `publish_camera:=false` launch argument. For example:
```bash
ros2 launch prisma_rover_navigation sim_navigation.launch.py publish_camera:=false
```
