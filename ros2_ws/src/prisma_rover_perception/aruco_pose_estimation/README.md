# aruco_pose_estimation

`aruco_pose_estimation` is a proprietary ROS 2 Python package (under the `prisma_rover_perception` directory) for estimating the 3D pose of detected ArUco markers by combining RGB and Depth camera streams.

## Main Features

* **3D Pose Estimation via Depth Alignment**: Segments the ArUco marker region inside the RGB image and uses a synchronized Depth image to extract the exact 3D centroid of the marker using ray-casting.
* **IPPE-Square Fallback (`solvePnP`)**: If the depth camera is inactive or the marker is out of the depth sensor's range, the node calculates the pose by solving the Perspective-n-Point (PnP) problem using camera calibration parameters.
* **Global Map Coordinate Projection**: Performs TF calls (`lookup_transform`) to transform the computed marker pose from the optical camera frame to the global `/map` frame.

## Package Structure

* **`launch/`**:
  * `aruco_pose_estimation.launch.py`: Loads the YAML parameters and starts the `aruco_node` with an initial stabilization delay of 10 seconds.
* **`config/`**:
  * `aruco_parameters.yaml`: Contains camera calibration settings, input/output topic configurations, target frames, and the ArUco dictionary ID (default: `DICT_5X5_250`).
* **`aruco_pose_estimation/`**:
  * `aruco_node.py`: Main ROS 2 node managing synchronized RGB/Depth image streams and TF lookup queries.
  * `pose_estimation.py`: Core logic for computing marker poses and 3D centroid extraction.
  * `utils.py`: Utility functions for drawing axes and mapping dictionary IDs.

## Main Topics

* **Subscribes to**:
  * `/camera/color/image_raw` (`sensor_msgs/msg/Image`): Input RGB images.
  * `/camera/depth/image_raw` (`sensor_msgs/msg/Image`): Input Depth images.
  * `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`): Camera intrinsic matrix.
* **Publishes to**:
  * `/aruco_poses` (`geometry_msgs/msg/PoseArray`): Array of marker poses for RViz visualization.
  * `/aruco_markers` (`prisma_rover_interfaces/msg/ArucoMarkers`): Custom message containing IDs and Poses.
  * `/aruco_image` (`sensor_msgs/msg/Image`): Annotated output images showing detections.
  * `/aruco_string` (`std_msgs/msg/String`): Formatted string with global `/map` coordinates.

## Key Dependencies

* `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`
* `prisma_rover_interfaces` (custom interfaces)
* `tf_transformations` (for quaternion conversions)
* `open3d` (3D processing library)
* OpenCV Python (`opencv-python` / `cv_bridge`)

## How to Use

```bash
# Compilation
colcon build --packages-select aruco_pose_estimation

# Launch the pose estimation node
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py use_sim_time:=true namespace:=prisma_rover
```
