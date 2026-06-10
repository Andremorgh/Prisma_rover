# prisma_rover_interfaces

`prisma_rover_interfaces` is the proprietary ROS 2 package that defines and consolidates custom message, service, and action types used throughout the **Prisma Rover** workspace.

## Main Features

* **ArUco Marker Messages (`ArucoMarkers.msg`)**: Structured message including an `std_msgs/Header`, a list of integer marker IDs, and a geometry `geometry_msgs/Pose` array representing the 3D poses of detected markers.
* **Roboclaw Messages (`RoboclawEncoderSteps.msg` and `RoboclawMotorVelocity.msg`)**: Defines data formats for wheel encoder telemetry and motor velocity target commands.

## Package Structure

* **`msg/`**:
  * `ArucoMarkers.msg`: Array of ArUco marker IDs and poses.
  * `RoboclawEncoderSteps.msg`: Encoder step readings from the Roboclaw controller for each wheel.
  * `RoboclawMotorVelocity.msg`: Target velocities for each Roboclaw motor channel.
  * `Dummy.msg`: Temporary placeholder message.
* **`CMakeLists.txt` / `package.xml`**: Configures the ROS 2 build interfaces (`rosidl_default_generators`) to compile the messages for C++, Python, and LISP.

## Key Dependencies

* `std_msgs`
* `geometry_msgs`
* `builtin_interfaces`

## How to Use

All other proprietary packages import and reference these message types. To compile the interfaces:

```bash
# Compilation
colcon build --packages-select prisma_rover_interfaces

# Verify the compiled message structures
ros2 interface show prisma_rover_interfaces/msg/ArucoMarkers
```
