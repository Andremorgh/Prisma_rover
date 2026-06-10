# Prisma Rover: Real Hardware Deployment Workspace

This is the official ROS 2 Humble workspace for the **Prisma Rover** physical robot, optimized for hardware deployment, autonomous 2D/3D navigation, SLAM mapping, and ArUco marker perception.

![Prisma Rover](docs/real_rover.png)

> [!NOTE]
> For more detailed technical specifications, changes log, and design notes, please refer to the [docs/](file:///home/andrea/Desktop/Prisma_rover/docs) folder.

---

## Table of Contents
1. [Workspace Architecture](#workspace-architecture)
2. [Hardware & Network Requirements](#hardware--network-requirements)
3. [Docker Integration & Build Setup](#docker-integration--build-setup)
4. [Installation & Build](#installation--build)
5. [Running on the Physical Robot](#running-on-the-physical-robot)
6. [Hardware Troubleshooting FAQ](#hardware-troubleshooting-faq)

---

## Workspace Architecture

The workspace is organized into modular ROS 2 packages under `ros2_ws/src/`, optimized for hardware execution:

### 1. Hardware Drivers & Bringup
* **[prisma_rover_bringup](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_bringup)**: Main launch files and parameter sets (like `livox_mid360.json` and `roboclaw.yaml`) to bring up the physical sensors, EKF, and motors.
* **[roboclaw_ros2](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/roboclaw_ros2)**: Low-level C++ motor driver and differential kinematics node to interface with the RoboClaw motor controller.
* **[prisma_rover_description](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_description)**: Robot URDF model defining link/joint frames (chassis, wheels, RealSense, LiDAR, IMU) for RViz TF coordinate publishing.

### 2. Localization & Navigation
* **[prisma_rover_localization](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_localization)**: Configures the Extended Kalman Filter (EKF) state estimation nodes, fusing real wheel odometry and IMU data.
* **[prisma_rover_navigation](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_navigation)**: Coordinates SLAM Toolbox configurations, RTAB-Map SLAM, and Nav2 costmap/path-planner parameters using `teb_local_planner`.

### 3. High-level Control & Perception
* **[prisma_rover_manager](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_manager)**: High-level C++ orchestrator coordinating task missions (goto, sweep coverage, return home).
* **[prisma_rover_explorer](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_explorer)**: frontier-based exploration and sweep coverage controllers.
* **[aruco_detector](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_perception/aruco_detector)**: Fast C++ OpenCV node to detect ArUco markers and broadcast their transforms.
* **[aruco_pose_estimation](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_perception/aruco_pose_estimation)**: Python node that aligns camera RGB and Depth data to estimate the 3D pose of detected markers.
* **[prisma_rover_interfaces](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_interfaces)**: Defines custom ROS 2 message structures (Roboclaw telemetry, ArUco markers).

---

## Hardware & Network Requirements

### 1. Ethernet LiDAR (Livox Mid-360)
The Livox Mid-360 LiDAR communicates over multicast Ethernet UDP packets. For the LiDAR driver node (`livox_ros_driver2_node`) to receive sensor packets:
* The robot's host network card connected to the LiDAR must be set to static IP **`192.168.1.50`** (subnet mask `255.255.255.0`).
* The physical LiDAR IP is configured as **`192.168.1.137`**.
* The Docker container must run in host networking mode (`--net=host`) so the multicast UDP traffic is routed directly into the container.

### 2. RoboClaw & USB Devices
The RoboClaw motor controller must be connected to the host PC via USB.
* By default, the driver expects the controller on serial port **`/dev/ttyACM0`** (or `/dev/ttyUSB0`).
* Ensure that the host user belongs to the `dialout` group (`sudo usermod -aG dialout $USER`).
* The Docker container is started in `--privileged` mode with `/dev` mounted, permitting the container to communicate with the serial device.

---

## Docker Integration & Build Setup

The workspace builds as a single target image (`prisma_rover:real`) based on `ros:humble`. It completely omits Gazebo simulation engines and heavy AI frameworks (like PyTorch and Ultralytics YOLO) to minimize runtime overhead and image size. It pre-installs:
* Livox SDK 2 and `livox_ros_driver2` (globally compiled in `/opt/livox_ws`)
* RealSense D435i camera driver (`realsense2_camera`)
* 2D RPLidar driver (`rplidar_ros`)
* Nav2, SLAM Toolbox, RTAB-Map, and Robot Localization.

---

## Installation & Build

### Steps
1. Build the hardware Docker image:
   ```bash
   ./docker_build.sh
   ```
2. Launch the container (privileged, device mapping, and host networking):
   ```bash
   ./docker_run.sh
   ```
3. Inside the container, compile the ROS 2 workspace:
   ```bash
   colcon build --symlink-install
   ```

---

## Running on the Physical Robot

Always run these launch commands **inside the running Docker container**.

### 1. Low-level Hardware Bringup
To start the motor drivers, differential kinematics, EKF odometry, RPLidar/Livox drivers, and the RealSense camera:
```bash
ros2 launch prisma_rover_bringup hardware_bringup.launch.py lidar_type:=3d camera:=true
```

### 2. Autonomous Navigation (with SLAM)
To run SLAM mapping, Nav2 trajectory planning, and the TF-to-Pose translator:
```bash
ros2 launch prisma_rover_navigation real_navigation.launch.py slam_type:=toolbox lidar_type:=3d
```
* Change `slam_type:=rtabmap` to use RTAB-Map SLAM instead of SLAM Toolbox.
* Change `lidar_type:=2d` if running with the 2D RPLidar instead of the 3D Livox.

### 3. ArUco Marker Tracker & Navigation
To run navigation and launch the ArUco marker perception node aligned to the RealSense camera image stream:
```bash
ros2 launch prisma_rover_navigation real_aruco_navigation.launch.py lidar_type:=3d
```

### 4. Manual Control (Teleop)
To manually drive the robot (e.g. from a second container terminal):
* **Keyboard Control**:
  ```bash
  ros2 run prisma_rover_teleop teleop_keyboard --ros-args -p cmd_vel_topic:=/prisma_rover/cmd_vel
  ```
* **Joystick Control (DualShock/Xbox)**:
  ```bash
  ros2 launch prisma_rover_teleop teleop.launch.py mode:=joy cmd_vel_topic:=/prisma_rover/cmd_vel
  ```

---

## Hardware Troubleshooting FAQ

### Q1: The Livox LiDAR is not publishing data inside Docker. How do I fix it?
1. Ping the LiDAR from the host: `ping 192.168.1.137`. If it doesn't respond, verify the Ethernet connection and your static IP configuration (`192.168.1.50`).
2. Verify that `docker_run.sh` was launched without custom network parameters. It must use `--net=host` to receive UDP packets.
3. Check the host firewall (e.g. `ufw status`). Multicast/UDP ports `56100`, `56200`, and `56300` must be open.

### Q2: Why does `roboclaw_node` report "Failed to open serial port"?
1. Verify that the RoboClaw is plugged in and check its port name: `ls -la /dev/ttyACM*` or `ls -la /dev/ttyUSB*`.
2. Check that the port matches the `serial_port` path defined in [roboclaw.yaml](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_bringup/config/roboclaw.yaml).
3. Ensure the host system permissions are correct (`sudo chmod 666 /dev/ttyACM0`).

### Q3: How do I share ROS 2 topics with a remote monitoring laptop (running RViz)?
By default, `docker_run.sh` sets `ROS_LOCALHOST_ONLY=1` to isolate DDS traffic. To allow external PCs on the network to receive topics:
1. Start the container with the environment variable set to 0:
   ```bash
   ROS_LOCALHOST_ONLY=0 ./docker_run.sh
   ```
2. Export `ROS_LOCALHOST_ONLY=0` on your remote monitoring PC.
3. Ensure both the robot PC and the monitoring laptop are on the same local network subnet and share the same `ROS_DOMAIN_ID` (default is `90`).
