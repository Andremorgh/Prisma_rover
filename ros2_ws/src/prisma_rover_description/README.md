# prisma_rover_description

`prisma_rover_description` is the proprietary ROS 2 package containing the kinematic, geometric, and visual models (URDF/Xacro) of the **Prisma Rover** robot.

## Main Features

* **Kinematic & Geometric Definition (Xacro)**: Defines the links and joints of the chassis, wheels, plates, and related joints.
* **Sensor Integration**: Conditionally attaches sensors such as the RGB-D camera (RealSense D435i), LiDAR (2D or 3D), and IMU to the robot model.
* **Robot State Publishing**: Calculates and broadcasts the fixed coordinate transforms (TF tree) between the various links of the robot.
* **Visualization (RViz2)**: Provides a pre-configured RViz setup to visually inspect the 3D model, sensor readings, and planned paths.

## Package Structure

* **`launch/`**:
  * `description.launch.py`: Parses the Xacro files to generate the URDF, starts the `robot_state_publisher` node, and optionally runs RViz2.
  * `rviz.rviz`: Pre-configured settings file for RViz2.
* **`urdf/`**:
  * `rover.xacro`: Main entry point file loading the robot.
  * `rover_macro.xacro`: Xacro macros defining link geometries, inertia tensors, and physical properties.
  * `rover_gazebo.xacro`: Gazebo plugins (diff_drive physics engine, joint state publishers, simulated IMU, LiDAR, and camera sensors).
  * `utilities.xacro`: Mathematical functions and reusable inertia macros.
  * `imu.urdf`: Minimal standalone definition of the IMU sensor.
* **`meshes/`**: Contains the 3D OBJ/MTL visual files used to render the robot parts in Gazebo and RViz.

## Parameters and Configuration

`description.launch.py` supports the following launch arguments:
* `use_sim_time` (default: `true`): Set to `true` in simulation.
* `namespace` (default: `""`): Namespace for the ROS 2 nodes and topics.
* `tf_prefix` (default: `""`): Prefix added to all frames in the TF tree (e.g., `prisma_rover/`).
* `rviz` (default: `false`): If set to `true`, launches the RViz2 graphical user interface.
* `publish_camera` (default: `true`): If set to `false`, excludes the camera from the URDF model to save computing resources.
* `lidar_type` (default: `3d`): Type of LiDAR model to include geometrically (`2d` or `3d`).

## How to Use

To compile and launch the robot state publisher:

```bash
# Compilation
colcon build --packages-select prisma_rover_description

# Example: Simple launch of the state publisher
ros2 launch prisma_rover_description description.launch.py

# Example: Launch with RViz2 visualization, TF prefix enabled, and camera model disabled to save resources
ros2 launch prisma_rover_description description.launch.py rviz:=true tf_prefix:=prisma_rover/ publish_camera:=false
```
