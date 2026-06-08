# aruco_detector

`aruco_detector` is a proprietary ROS 2 C++ package (under the `prisma_rover_perception` directory) dedicated to real-time ArUco fiducial marker detection from camera video streams.

## Main Features

* **ArUco Marker Detection (OpenCV)**: Identifies ArUco markers within video frames and calculates the 3D position and orientation of each marker relative to the camera frame.
* **TF Broadcaster**: Automatically broadcasts dynamic coordinate frame transforms (`tf`) for each detected marker, enabling other nodes to query marker coordinates relative to the robot base.
* **Detailed Info Publishing**: Publishes annotated camera frames showing bounding boxes and 3D coordinate axes drawn on top of markers, alongside lists of detected marker IDs and their transforms.

## Package Structure

* **`launch/`**:
  * `detector_launch.py`: Launch script to start the `rover_aruco_detector` node, configuring parameters for the ArUco dictionary, physical marker size, and video input topics.
* **`src/`**:
  * `aruco_detector_node.cpp`: Main C++ node implementing image subscription, OpenCV image processing, and TF broadcasting.

## Main Topics

* **Subscribes to**:
  * `camera` (default: `/prisma_rover/camera/color/image_raw`): Input RGB video topic.
  * `camera_info` (default: `/prisma_rover/camera/color/camera_info`): Intrinsic camera calibration info.
* **Publishes to**:
  * `[node_name]/result_img`: Annotated image stream showing bounding boxes and axes.
  * `[node_name]/tf_list`: List of computed spatial transforms.
  * `[node_name]/aruco_list`: Array of integer IDs for detected markers.

## Key Dependencies

* `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`
* `cv_bridge` (for bridging ROS images to OpenCV formats)
* OpenCV with `aruco` module (computer vision library)
* `tf2`, `tf2_ros`, `image_transport`

## How to Use

To compile and launch the ArUco detector node:

```bash
# Compilation
colcon build --packages-select aruco_detector

# Launch the detector node
ros2 launch aruco_detector detector_launch.py camera:=/prisma_rover/camera/color/image_raw camera_info:=/prisma_rover/camera/color/camera_info publish_tf:=true
```
