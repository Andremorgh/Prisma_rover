# prisma_rover_teleop

`prisma_rover_teleop` is the proprietary ROS 2 package responsible for manual teleoperation control of the **Prisma Rover** robot using a keyboard or a physical controller/joystick.

## Main Features

* **Keyboard Control (`teleop_keyboard.py`)**: Steers the robot by publishing `geometry_msgs/msg/Twist` messages on the `/cmd_vel` topic using W-A-S-D keys (with additional hotkeys for linear and angular speed tuning).
* **Gamepad/Joystick Control**: Interfaces with the standard `joy` and `teleop_twist_joy` ROS 2 nodes to map analog sticks and buttons of a controller (e.g., Xbox, Playstation, or Logitech controllers) into velocity command outputs.
* **Namespacing Support**: Resolves and maps the control commands under the correct robot namespace (e.g., `/prisma_rover/cmd_vel`).

## Package Structure

* **`launch/`**:
  * `teleop.launch.py`: Launch script for joystick teleoperation, loading key associations and mapping parameters.
* **`config/`**:
  * `teleop_params.yaml`: Configures analog channel mappings (linear/angular axes) and the safety deadman button (e.g., LB/L1) to prevent accidental movements.
* **`prisma_rover_teleop/`**:
  * `teleop_keyboard.py`: Standalone Python script for terminal-based character teleoperation.

## Key Dependencies

* `joy` (standard ROS 2 joystick driver package)
* `teleop_twist_joy` (joy event to Twist translator)
* `rclpy`, `geometry_msgs`

## How to Use

### 1. Keyboard Teleoperation

To start the keyboard teleoperation node (run in an active terminal window):

```bash
# Compilation
colcon build --packages-select prisma_rover_teleop

# Start the keyboard node (with namespace)
ros2 run prisma_rover_teleop teleop_keyboard --ros-args -r __ns:=/prisma_rover
```

### 2. Gamepad/Joystick Teleoperation

Connect your USB controller/gamepad to the host PC and run:

```bash
# Launch the joystick bridge
ros2 launch prisma_rover_teleop teleop.launch.py namespace:=prisma_rover
```
*Note: Hold down the safety deadman button (e.g., LB/L1) on the gamepad to enable motor command transmission.*
