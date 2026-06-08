#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import open3d as o3d
import numpy as np
from cv_bridge import CvBridge
import cv2
import message_filters
from ultralytics import YOLO
from shapely.geometry import Polygon
from sklearn.cluster import DBSCAN
import time
import struct
import ros2_numpy

# Color and ID values
COCO_CATEGORIES = [
    {"id": 24, "name": "backpack"},
    {"id": 28, "name": "suitcase"},
    {"id": 39, "name": "bottle"},
    {"id": 41, "name": "cup"},
    {"id": 56, "name": "chair"},
    {"id": 63, "name": "laptop"},
    {"id": 64, "name": "mouse"},
    {"id": 65, "name": "remote"},
    {"id": 66, "name": "keyboard"},
    {"id": 67, "name": "cell phone"},
    {"id": 75, "name": "vase"},
]

class Realsense(Node):

    def __init__(self):
        super().__init__('realsense')

        # Frame publisher
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.static_transform_stamped = TransformStamped()

        # Realsense subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, 'color/image_raw')
        self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, 'depth/color/points')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub],
            queue_size=1,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # TF listener
        self.target_frame = self.declare_parameter('target_frame', 'prisma_rover/camera_rgb_optical_frame').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO setup
        from ament_index_python.packages import get_package_share_directory
        import os
        try:
            pkg_share = get_package_share_directory('obj_detection')
            model_path = os.path.join(pkg_share, 'scripts', 'yolo11n-seg.pt')
            if not os.path.exists(model_path):
                model_path = "/home/user/ros2_ws/src/obj_detection/scripts/yolo11n-seg.pt"
        except Exception:
            model_path = "/home/user/ros2_ws/src/obj_detection/scripts/yolo11n-seg.pt"
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.category_dict = {cat['id']: cat['name'] for cat in COCO_CATEGORIES}

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.05, self.camera)
        self.color_image = []
        self.pc = []
        self.detect = True

    def sync_callback(self, image_msg, cloud_msg):
        print('Data received')
        # ---------- POINT CLOUD ----------
        # Convert the PointCloud2 message to an array
        pc = ros2_numpy.point_cloud2.point_cloud2_to_array(cloud_msg)
        self.pc = pc['xyz']

        # ---------- YOLO ----------
        # Image transformations to fit YOLO format
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        self.color_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def camera(self):
        if np.array(self.pc).any() and np.array(self.color_image).any() and self.detect:

            from_frame_rel = self.target_frame
            to_frame_rel = 'prisma_rover/map'

            try:
                tf = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info('Could not transform frames.')
                return

            # YOLO inference limited to the target object: 39 (bottle)
            results = self.model.track(self.color_image, persist=True, device="cpu", classes=[39])

            # Point cloud cropping using detected masks
            if results[0].masks is not None:
                height, width = self.color_image.shape[:2]
                masks = results[0].masks.xy
                classes = results[0].boxes.cls

                buffer = bytearray()
                point_struct = struct.Struct("<fffBBBB")

                for i, mask in enumerate(masks):
                    # Selecting points corresponding to object i
                    binary_mask = np.zeros(self.color_image.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
                    indices = np.argwhere(binary_mask == 1)
                    indices_1d = indices[:, 0] * width + indices[:, 1]
                    cropped_vtx = self.pc[indices_1d[::20]]
                    pcmask = (cropped_vtx[:,0] != 0) | (cropped_vtx[:,1] != 0) | (cropped_vtx[:,2] != 0) # Removing points placed in origin
                    cropped_vtx = cropped_vtx[pcmask,:]

                    if len(cropped_vtx)>30:
                        try:
                            # Create o3d point cloud for 3d OBB
                            cloudo3d = o3d.geometry.PointCloud()
                            pcd_points = np.array([(point[0], point[1], point[2]) for point in cropped_vtx], dtype=np.float32)
                            cloudo3d.points = o3d.utility.Vector3dVector(pcd_points)

                            # Filter point cloud
                            cloudo3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05,max_nn=30))
                            cloudo3d.orient_normals_consistent_tangent_plane(100)
                            normal_threshold = 0.4
                            normals = np.asarray(cloudo3d.normals)
                            indices = np.where(np.abs(normals[:,2])>normal_threshold)[0]
                            filt_pcd = cloudo3d.select_by_index(indices)

                            labels = np.array(filt_pcd.cluster_dbscan(eps=0.05,min_points=30, print_progress=False))
                            unique_labels, counts = np.unique(labels[labels !=-1], return_counts=True)
                            largest_cluster_label = unique_labels[np.argmax(counts)]
                            object_indices = np.where(labels == largest_cluster_label)[0]
                            filtered_pcd = filt_pcd.select_by_index(object_indices)

                            filtered_pcd.translate((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                            R = o3d.geometry.get_rotation_matrix_from_quaternion((tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z))
                            filtered_pcd.rotate(R, center=(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                            
                            # Extract point cloud centroid and publish it as a frame
                            points = np.asarray(filtered_pcd.points)
                            centroid = points.mean(axis=0)

                            self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
                            self.static_transform_stamped.header.frame_id = 'prisma_rover/map'
                            self.static_transform_stamped.child_frame_id = 'bottle'

                            self.static_transform_stamped.transform.translation.x = centroid[0]
                            self.static_transform_stamped.transform.translation.y = centroid[1]
                            self.static_transform_stamped.transform.translation.z = 0.0

                            self.static_transform_stamped.transform.rotation.x = 0.0
                            self.static_transform_stamped.transform.rotation.y = 0.0
                            self.static_transform_stamped.transform.rotation.z = 0.0
                            self.static_transform_stamped.transform.rotation.w = 1.0

                            self.tf_static_broadcaster.sendTransform(self.static_transform_stamped)

                            self.detect = False

                        except:
                            continue

            self.color_image = []
            self.pc = []

def main(args=None):
    rclpy.init(args=args)

    realsense = Realsense()
    rclpy.spin(realsense)

    realsense.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
