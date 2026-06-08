# prisma_rover_bringup

`prisma_rover_bringup` is the proprietary ROS 2 package responsible for the coordinated launch of real hardware drivers and onboard sensors for the **Prisma Rover** robot.

## Main Features

* **Motor Drivers (Roboclaw)**: Initiates serial communication with the Basic Micro Roboclaw motor controllers.
* **Differential Kinematics & Odometry**: Computes conversion between linear/angular velocities and motor commands, publishing raw wheel odometry (`/prisma_rover/odom/wheels`).
* **LiDAR Sensor Integration**: Conditionally supports 2D (RPLidar S2) or 3D (Livox Mid-360) sensors.
* **Intel RealSense Camera**: Starts the physical RGB-D camera driver, enabling depth alignment and PointCloud generation.
* **Extended Kalman Filter (EKF)**: Fuses raw wheel odometry and IMU data to produce a stable local state estimation.

## Package Structure

* **`config/`**:
  * `roboclaw.yaml`: Serial settings, encoder counts per meter, track width, and motor kinematic parameters.
  * `hardware_ekf.yaml`: EKF state estimation configuration for real sensor fusion.
  * `livox_mid360.json`: Ethernet/IP connection settings for the Livox 3D LiDAR.
* **`launch/`**:
  * `hardware_bringup.launch.py`: Main orchestrator launch file to bring up all physical nodes and sensors on the real robot.

## Nodes Managed by Hardware Bringup

1. **`roboclaw_node`** (`roboclaw_ros2`): Direct serial communication with the Roboclaw hardware.
2. **`diffdrive_node`** (`roboclaw_ros2`): Differential kinematics calculations and raw wheel odom publisher.
3. **`ekf_filter_node`** (`robot_localization`): Fuses onboard sensor streams to broadcast the `/odom` -> `/base_footprint` transform.
4. **`rplidar_node`** (`rplidar_ros`): Driver node for 2D RPLidar (active when `lidar_type:=2d`).
5. **`livox_ros_driver2_node`** (`livox_ros_driver2`): Driver node for 3D Livox (active when `lidar_type:=3d`).
6. **`camera`** (`realsense2_camera_node`): Driver for the physical RealSense D435i camera.

## Key Dependencies

* `roboclaw_ros2` (Roboclaw Driver)
* `robot_localization` (EKF)
* `rplidar_ros` (RPLidar Driver)
* `livox_ros_driver2` (Livox Driver)
* `realsense2_camera` (RealSense Driver)
* `prisma_rover_description` (URDF Kinematic Model)

## How to Use

To launch the onboard sensors and drivers on the real robot:

```bash
# Compilation
colcon build --packages-select prisma_rover_bringup

# Example: Launch with Livox Mid-360 3D LiDAR and Camera enabled
ros2 launch prisma_rover_bringup hardware_bringup.launch.py lidar_type:=3d publish_camera:=true

# Example: Alternative launch with 2D RPLidar and camera disabled
ros2 launch prisma_rover_bringup hardware_bringup.launch.py lidar_type:=2d publish_camera:=false
```
