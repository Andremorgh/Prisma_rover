# prisma_rover_localization

`prisma_rover_localization` is the proprietary ROS 2 package responsible for launching and configuring the Extended Kalman Filter (EKF) node to estimate the state and pose of the **Prisma Rover** robot.

## Main Features

* **Sensor Fusion (EKF)**: Fuses raw wheel odometry (`/prisma_rover/odom/wheels`) with data from the onboard Inertial Measurement Unit (IMU) (`/prisma_rover/imu/data`).
* **Stable Odometry Estimation**: Reduces pose drift from wheel slippage by broadcasting a stable `/odom` frame and its dynamic transform (`/odom` -> `/base_footprint`).
* **Namespacing Support**: Fully configured for namespaced executions to prevent topic and frame name collisions in multi-robot setups.

## Package Structure

* **`config/`**:
  * `localization_ekf.yaml`: Contains covariance matrices, sensor boolean masking arrays (configuring variables like x, y, z, roll, pitch, yaw, linear/angular velocities to fuse for each sensor), and robot base frame parameters.
* **`launch/`**:
  * `localization.launch.py`: Launch script that spawns the `ekf_node` from `robot_localization`, loading YAML configurations and remapping parameters based on namespace and sim time settings.

## Key Dependencies

* `robot_localization` (Standard ROS 2 package for EKF)
* `rclcpp`, `sensor_msgs`, `nav_msgs`

## How to Use

The EKF node is typically launched by the main navigation orchestrator, but can be started independently as follows:

```bash
# Compilation
colcon build --packages-select prisma_rover_localization

# Launch the EKF filter in simulation
ros2 launch prisma_rover_localization localization.launch.py use_sim_time:=true namespace:=prisma_rover
```
