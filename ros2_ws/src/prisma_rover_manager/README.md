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
    * `"goto"`: Sends the robot to predefined target coordinates or TF frames (e.g., `'goto,1.5,2.0,0.0'` or `'goto,camera_link'`).
    * `"coverage"` / `"cover"`: Initiates systematic boustrophedon sweep coverage.
    * `"return"` / `"return_to_base"`: Sends the robot back to the predefined home coordinate.
    * `"stop"` / `"emergency_stop"`: Instantly cancels active Nav2 goals and publishes zero velocity commands to block the robot in place.
    * `"cancel"` / `"cancel_goal"`: Cancels the active Nav2 goal.
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
ros2 topic pub --once /prisma_rover/seed_pdt_rover/command std_msgs/msg/String "{data: 'return'}"

# Example: Publish emergency stop command
ros2 topic pub --once /prisma_rover/seed_pdt_rover/command std_msgs/msg/String "{data: 'stop'}"
```

## Adding New Commands

To extend the `prisma_rover_manager` with new custom commands, follow these steps:

1. **Locate the parser in C++**:
   Open [rover_manager.cpp](file:///home/andrea/Desktop/Prisma_rover/ros2_ws/src/prisma_rover_manager/src/rover_manager.cpp) and navigate to the `RoverManager::execute_command` method.
   The incoming string is tokenized into a vector `cv` by `instance2vector` (where `cv[0]` is the command name `op`, and `cv[1]`, `cv[2]`, etc., are arguments).

2. **Add the execution block**:
   Add a new `else if` branch for your command. For example:
   ```cpp
   else if (op == "my_new_command") {
       // Retrieve parameters from cv if needed
       // Implement custom ROS 2 actions or velocity overrides here
       command_running = true; // Set to true if running asynchronously
   }
   ```

3. **Rebuild the package**:
   ```bash
   colcon build --packages-select prisma_rover_manager
   ```
