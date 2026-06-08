# obj_detection

`obj_detection` is a proprietary ROS 2 package dedicated to object detection, semantic segmentation, and 3D coordinate projection of detected objects to build a semantic map of the environment.

## Main Features

* **Detection & Segmentation (YOLOv11)**: Detects and segments objects in real time from camera RGB images (either from physical RealSense or simulation) using Ultralytics YOLOv11 models.
* **3D Projection via Depth Alignment**: Associates pixel coordinates of detected objects with depth camera data to project the object's 3D position relative to the camera's optical frame.
* **Semantic Mapping & JSON Database**: Calculates global coordinates in the `/map` frame using TF transforms and saves objects into a local JSON database (`objects_map.json`), avoiding close spatial duplicates and computing spatial proximity relations (`relNEARBY`).

## Package Structure

* **`launch/`**:
  * `perception_mapping.launch.py`: Launches the object detection and mapping node, configuring topic parameters and thresholds.
  * `realsense_launch.py`: Configures and launches the node for the physical Intel RealSense camera.
* **`scripts/`**:
  * `detect_objects_rs.py`: Main Python node for object detection and 3D projection.
  * `objects_map.py`: Auxiliary node for managing and saving objects in the local JSON database.
  * `coco_categories.json`: Dictionary of COCO categories supported by YOLO.
* **`src/`**:
  * `simple_node.cpp`: Minimal C++ test node.

## Key Dependencies

* `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`
* `cv_bridge` (for converting ROS images to OpenCV format)
* `ultralytics` (for YOLOv11)
* Python libraries: `numpy`, `opencv-python`

## Parameters and Configuration

The main parameters configurable via `perception_mapping.launch.py` include:
* `camera_topic` (default: `/prisma_rover/camera/color/image_raw`): Input RGB topic.
* `depth_topic` (default: `/prisma_rover/camera/depth/image_raw`): Input Depth topic.
* `camera_info_topic` (default: `/prisma_rover/camera/color/camera_info`): Intrinsic camera info topic.
* `confidence_threshold` (default: `0.5`): YOLO detection confidence threshold.

## How to Use

To compile and launch the node in simulation or real hardware environments:

```bash
# Compilation
colcon build --packages-select obj_detection

# Launch the perception and semantic mapping node
ros2 launch obj_detection perception_mapping.launch.py
```
