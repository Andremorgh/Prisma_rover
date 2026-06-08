# prisma_rover_sim

`prisma_rover_sim` is the proprietary ROS 2 package responsible for launching and managing the 3D simulation environment based on **Ignition Gazebo (Fortress)** for the **Prisma Rover** robot.

## Main Features

* **3D World Loading (SDF)**: Provides pre-configured 3D worlds containing mazes, obstacles, and ArUco markers distributed in the environment to validate navigation, avoidance, and RL algorithms.
* **Robot Spawning**: Spawns the URDF/Xacro robot model inside the physics simulator at configurable initial coordinates.
* **Bidirectional Bridge (ros_gz_bridge)**: Handles data exchange between Ignition Gazebo and ROS 2 (publishing motor command velocities `cmd_vel`, receiving IMU telemetry, LiDAR scans, RGB-D camera images, and simulation ground-truth odometry).

## Package Structure

* **`launch/`**:
  * `sim.launch.py`: Main launch file that runs the Gazebo simulator server/client, starts the topic bridges, and spawns the robot.
* **`worlds/`**: Contains the SDF definition files for simulation worlds.
* **`models/`**: Additional 3D models and ArUco marker asset definitions loaded in simulation.

## Key Dependencies

* `ros_gz_sim`, `ros_gz_bridge` (Ignition Gazebo - ROS 2 integration)
* `prisma_rover_description` (to retrieve the Xacro robot description)
* `rclcpp`, `std_msgs`, `sensor_msgs`

## How to Use

To compile and launch the simulation environment:

```bash
# Compilation
colcon build --packages-select prisma_rover_sim

# Run the simulator with GUI enabled and namespace configured
ros2 launch prisma_rover_sim sim.launch.py headless:=false namespace:=prisma_rover
```
