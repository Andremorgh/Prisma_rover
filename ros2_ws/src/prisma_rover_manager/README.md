# prisma_rover_manager

`prisma_rover_manager` is a proprietary ROS 2 package in C++ (previously named `rover_manager`) that serves as a high-level state and mission coordinator for the **Prisma Rover**.

## Main Features

* **Robot State Management**: Receives mission commands in string format and manages the internal state transitions of the robot (e.g., IDLE, GOING_TO_GOAL, COVERAGE, RETURNING_HOME).
* **Nav2 Interface**: Implements action clients for the Nav2 navigation server (`/prisma_rover/navigate_to_pose`) to autonomously drive the robot to target coordinates.
* **Sweep Coverage Support**: Integrates with the sweep coverage interface to initiate systematic exploration patterns.
* **Automatic Return to Home**: Allows aborting any active mission to safely navigate the robot back to its starting coordinates (`0.0, 0.0`).

## Package Structure

* **`launch/`**:
  * `manager.launch.py`: Starts the `rover_manager` node, configuring namespace and simulation time parameters.
* **`src/`**:
  * `rover_manager.cpp`: Main C++ source code for the mission coordinator node.
* **`include/`**: Header file containing class and method declarations for the manager.
* **`rover.rviz`**: Pre-configured RViz settings file for debugging the state manager.

## Main Topics and Services

* **Subscribes to**:
  * `/prisma_rover/seed_pdt_rover/command` (`std_msgs/msg/String`): Input channel for mission commands. Supported commands:
    * `"goto"`: Sends the robot to predefined target coordinates.
    * `"coverage"`: Initiates systematic sweep coverage.
    * `"home"`: Sends the robot back to the origin pose.
* **Publishes to**:
  * `/prisma_rover/seed_pdt_rover/state` (`std_msgs/msg/String`): Broadcasts the active state of the manager.

## Key Dependencies

* `rclcpp` (ROS 2 C++ client library)
* `std_msgs`, `geometry_msgs`
* `nav2_msgs` (for Nav2 action interfaces)

## How to Use

To compile and launch the mission manager:

```bash
# Compilation
colcon build --packages-select prisma_rover_manager

# Launch the mission manager (in simulation with namespace)
ros2 launch prisma_rover_manager manager.launch.py use_sim_time:=true namespace:=prisma_rover

# Example: Publish command to send the robot back home via terminal
ros2 topic pub --once /prisma_rover/seed_pdt_rover/command std_msgs/msg/String "{data: 'home'}"
```
