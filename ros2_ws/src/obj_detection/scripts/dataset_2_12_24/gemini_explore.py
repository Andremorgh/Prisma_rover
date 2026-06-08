#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge
import json
import os

#ros2 topic pub /command std_msgs/msg/Int32 "{data: 1}" --once

class CameraCommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        
        self.bridge = CvBridge()
        self.image = None
        self.poses = []
        self.image_counter = 0

        self.target_frame = "rover/map"
        self.source_frame = "rover/base_link"

        self.camera_sub = self.create_subscription(Image, "/rover_camera/color/image_raw", self.image_callback, 1)
        self.command_sub = self.create_subscription(Int32, "/command", self.command_callback, 1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("CameraCommandNode initialized.")

    def image_callback(self, msg):
        self.image = msg

    def command_callback(self, msg):
        if msg.data == 1 and self.image is not None:
            pose = self.get_pose()
            if pose:
                saveFlag = self.save_image(self.image)
                if saveFlag==True:
                    self.poses.append(pose)
        else:
            self.get_logger().info("Cannot save pose, information missing.")
                
    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame,rclpy.time.Time())
            pose = {
                "translation": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                },
                "rotation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w,
                }
            }
            self.get_logger().info(f"Pose recorded: {pose}")
            return pose
        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Transform lookup failed: {e}")
            return None

    def save_image(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            filename = f"/home/user/ros2_ws/src/obj_detection/scripts/dataset/image_{self.image_counter:04d}.png"
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Image saved as {filename}")
            self.image_counter += 1
            return True
        except Exception as e:
            self.get_logger().info(f"Failed to save image: {e}")
            return False

    def shutdown_hook(self):
        self.get_logger().info("Shutting down...")
        try:
            with open("/home/user/ros2_ws/src/obj_detection/scripts/dataset/poses.json", "w") as json_file:
                json.dump(self.poses, json_file, indent=4)
            self.get_logger().info("Poses saved to poses.json")
        except Exception as e:
            self.get_logger().info(f"Failed to save poses: {e}")

def main(args=None):
    rclpy.init(args=args)
    command_node = CameraCommandNode()

    try:
        rclpy.spin(command_node)
    except KeyboardInterrupt:
        pass
    finally:
        command_node.shutdown_hook()
        command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()