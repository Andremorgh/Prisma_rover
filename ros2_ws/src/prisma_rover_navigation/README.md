# prisma_rover_navigation

`prisma_rover_navigation` is the proprietary ROS 2 package responsible for launching the autonomous navigation (Nav2) and mapping (SLAM) stack for the **Prisma Rover** robot.

## Main Features

* **Flexible SLAM Mapping**: Supports two SLAM backends selectable at runtime:
  * **SLAM Toolbox**: Online asynchronous 2D map generation using laser scans.
  * **RTAB-Map (3D SLAM)**: Visual or 3D LiDAR-based SLAM (ICP loop closure) for accurate 3D occupancy grid mapping.
* **2D & 3D LiDAR Support**: Enables using a direct 2D LiDAR scan or projecting a 3D PointCloud2 stream (e.g., from Livox Mid-360) to a 2D scan using the `pointcloud_to_laserscan` node.
* **Nav2 Path Planning & Control**: Launches the Nav2 stack, loading local/global costmaps, path planners, and local trajectory controllers based on **TEB Local Planner** (`teb_local_planner`).
* **Test Scenarios & Demos**: Provides pre-configured launch files to verify navigation integrated with YOLOv11 or ArUco markers.

## Package Structure

* **`launch/`**:
  * `sim_navigation.launch.py`: Main orchestrator launch file that starts the Gazebo simulation, EKF state estimation, SLAM, and Nav2.
  * `slam.launch.py`: Configures and launches SLAM Toolbox or RTAB-Map with the appropriate LiDAR sensor settings.
  * `navigation.launch.py`: Spawns the Nav2 servers (lifecycle manager, planner, controller, recovery).
  * `aruco_navigation.launch.py`: Configures navigation integrated with ArUco marker tracking.
  * `yolo_navigation.launch.py`: Configures navigation integrated with YOLOv11 object detection.
* **`params/`**:
  * `nav2_params.yaml`: Complete parameter set for Nav2 servers, costmaps, and detailed TEB local planner configuration.
  * `mapper_params_online_async.yaml`: Configurations for SLAM Toolbox.
  * `rtabmap.yaml`: Configurations for RTAB-Map (ICP parameters, LiDAR mode, pointcloud scan subscription).

## Key Dependencies

* `nav2_bringup`, `nav2_msgs`, `nav2_controller`, `nav2_planner`
* `slam_toolbox`, `rtabmap_slam`
* `teb_local_planner` (Local path planner)
* `pointcloud_to_laserscan` (3D -> 2D LiDAR projection)
* `prisma_rover_sim` (for simulation physics)
* `prisma_rover_localization` (for EKF pose estimation)

## How to Use

The package provides multiple ways to launch the navigation stack. Below are some examples:

```bash
# Compilation
colcon build --packages-select prisma_rover_navigation

# 1. Full launch in simulation (SLAM Toolbox + 2D LiDAR)
ros2 launch prisma_rover_navigation sim_navigation.launch.py slam_type:=toolbox lidar_type:=2d

# 2. Full launch in simulation (RTAB-Map + 3D LiDAR)
ros2 launch prisma_rover_navigation sim_navigation.launch.py slam_type:=rtabmap lidar_type:=3d

# 3. Launch the ArUco navigation demo
ros2 launch prisma_rover_navigation aruco_navigation.launch.py headless:=true rviz:=false
```
