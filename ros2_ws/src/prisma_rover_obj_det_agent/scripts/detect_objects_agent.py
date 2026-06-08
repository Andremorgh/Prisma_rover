#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Polygon as ROSPolygon
from geometry_msgs.msg import Point32, Pose
from prisma_rover_interfaces.msg import DetectedObject, DetectedObjectsList
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import message_filters
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

####
from ultralytics.models.yolo.segment import SegmentationPredictor
from ultralytics.utils import ASSETS
####

from shapely.geometry import Polygon
from sklearn.cluster import DBSCAN
from ultralytics import YOLO
import open3d as o3d
import numpy as np
import ros2_numpy
import struct
import time
import json
import cv2

COCO_CATEGORIES = [
    {"color": [255, 179, 240], "id": 24, "name": "backpack"},
    {"color": [0, 125, 92], "id": 25, "name": "umbrella"},
    {"color": [209, 0, 151], "id": 26, "name": "handbag"},
    {"color": [188, 208, 182], "id": 27, "name": "tie"},
    {"color": [0, 220, 176], "id": 28, "name": "suitcase"},
    {"color": [78, 180, 255], "id": 32, "name": "sports ball"},
    {"color": [197, 226, 255], "id": 39, "name": "bottle"},
    {"color": [171, 134, 1], "id": 40, "name": "wine glass"},
    {"color": [109, 63, 54], "id": 41, "name": "cup"},
    {"color": [207, 138, 255], "id": 42, "name": "fork"},
    {"color": [151, 0, 95], "id": 43, "name": "knife"},
    {"color": [9, 80, 61], "id": 44, "name": "spoon"},
    {"color": [84, 105, 51], "id": 45, "name": "bowl"},
    {"color": [74, 65, 105], "id": 46, "name": "banana"},
    {"color": [166, 196, 102], "id": 47, "name": "apple"},
    {"color": [208, 195, 210], "id": 48, "name": "sandwich"},
    {"color": [255, 109, 65], "id": 49, "name": "orange"},
    {"color": [153, 69, 1], "id": 56, "name": "chair"},
    {"color": [3, 95, 161], "id": 57, "name": "couch"},
    {"color": [119, 0, 170], "id": 59, "name": "bed"},
    {"color": [0, 182, 199], "id": 60, "name": "dining table"},
    {"color": [0, 165, 120], "id": 61, "name": "toilet"},
    {"color": [183, 130, 88], "id": 62, "name": "tv"},
    {"color": [95, 32, 0], "id": 63, "name": "laptop"},
    {"color": [130, 114, 135], "id": 64, "name": "mouse"},
    {"color": [110, 129, 133], "id": 65, "name": "remote"},
    {"color": [166, 74, 118], "id": 66, "name": "keyboard"},
    {"color": [219, 142, 185], "id": 67, "name": "cell phone"},
    {"color": [79, 210, 114], "id": 68, "name": "microwave"},
    {"color": [178, 90, 62], "id": 69, "name": "oven"},
    {"color": [65, 70, 15], "id": 70, "name": "toaster"},
    {"color": [127, 167, 115], "id": 71, "name": "sink"},
    {"color": [59, 105, 106], "id": 72, "name": "refrigerator"},
    {"color": [142, 108, 45], "id": 73, "name": "book"},
    {"color": [196, 172, 0], "id": 74, "name": "clock"},
    {"color": [95, 54, 80], "id": 75, "name": "vase"},
    {"color": [128, 76, 255], "id": 76, "name": "scissors"},
    {"color": [201, 57, 1], "id": 77, "name": "teddy bear"},
    {"color": [246, 0, 122], "id": 78, "name": "hair drier"},
    {"color": [191, 162, 208], "id": 79, "name": "toothbrush"},
]

colors = {
        'red': [255, 0, 0],
        'green': [0, 255, 0],
        'blue': [0, 0, 255],
        'purple': [255,0,255], # Magenta
        'yellow': [255, 255, 0],
        'gray': [255,255,255] # White
    }
colors_array = np.array(list(colors.values()))

class Realsense(Node):

    def __init__(self):
        super().__init__('realsense')

        self.get_logger().info('Initializing node...')

        # Publisher for sending the list of detected objects and their characteristics
        self.pub = self.create_publisher(DetectedObjectsList, 'objects_detected', 10)

        # Declare parameters
        self.declare_parameter('image_topic', 'color/image_raw')
        self.declare_parameter('pointcloud_topic', 'depth/color/points')
        self.declare_parameter('target_frame', 'prisma_rover/camera_color_optical_frame')
        self.declare_parameter('map_frame', 'prisma_rover/map')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        # Realsense subscriptions
        qos_profile_pc = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=3, reliability=ReliabilityPolicy.RELIABLE)
        qos_profile_image = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=6, reliability=ReliabilityPolicy.RELIABLE)
        self.image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=qos_profile_image)
        self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, pointcloud_topic, qos_profile=qos_profile_pc)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub],
            queue_size=5,
            slop=1.0
        )
        self.ts.registerCallback(self.sync_callback)
        self.color_image = []
        self.pc = []

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO setup with robust path resolver
        import os
        def resolve_model_path(filename):
            # Try script directory
            path = os.path.join(os.path.dirname(__file__), filename)
            if os.path.exists(path):
                return path
            # Try workspace folder structure
            ws_path = f"/home/andrea/Desktop/Quantum_obj_rover/ros2_ws/src/obj_agent/scripts/{filename}"
            if os.path.exists(ws_path):
                return ws_path
            return filename

        yolo_model_path = resolve_model_path("yolo11s-seg.pt")
        self.model = YOLO(yolo_model_path)

        #args = dict(model="yolo11n-seg.pt", source=ASSETS)
        #self.predictor = SegmentationPredictor(overrides=args)


        self.category_dict = {cat['id']: cat['name'] for cat in COCO_CATEGORIES}

        self.bridge = CvBridge()

        # FUnction to process and publish the visual information
        self.timer = self.create_timer(0.8, self.camera)
        self.COLOR_TO_IDX = {"red": 11, "green": 12, "blue": 13, "purple": 14, "yellow": 15, "gray": 16}

        self.get_logger().info('Done!')

    def sync_callback(self, image_msg, cloud_msg):
        # print('Data received')
        pc = ros2_numpy.point_cloud2.point_cloud2_to_array(cloud_msg)
        self.pc = pc['xyz']
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        self.color_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def detect_and_publish(self, results):
        from_frame_rel = self.target_frame
        to_frame_rel = self.map_frame
        
        try:
            tf = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
            print("TF")
        except TransformException as ex:
            self.get_logger().info('Could not transform frames.')
            return
        
        if results[0].masks is not None:
            print(results[0].probs)
            detected_objects = DetectedObjectsList()
            objetos = []

            _, width = self.color_image.shape[:2]
            masks = results[0].masks.xy 
            classes = results[0].boxes.cls
            track_ids = results[0].boxes.id

            for i, mask in enumerate(masks):
                # Selecting points corresponding to object i
                binary_mask = np.zeros(self.color_image.shape[:2], dtype=np.uint8)
                cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
                indices = np.argwhere(binary_mask == 1)
                indices_1d = indices[:, 0] * width + indices[:, 1]
                cropped_vtx = self.pc[indices_1d[::10]]
                pcmask = (cropped_vtx[:,0] != 0) | (cropped_vtx[:,1] != 0) | (cropped_vtx[:,2] != 0) # Removing points placed in origin
                cropped_vtx = cropped_vtx[pcmask,:]

                # Only considering point clouds with more than 10 points - if not open3d functions tend to fail
                if len(cropped_vtx)>10:
                    try:
                        # Selecting the closest color using the pixel values inside the object crop
                        selected_colors = self.color_image[indices[:, 0], indices[:, 1]]
                        avg_color = np.mean(selected_colors, axis=0)
                        avg_color=np.flip(avg_color)
                        distances = np.linalg.norm(colors_array - avg_color, axis=1)
                        closest_color_name = list(colors.keys())[np.argmin(distances)]
                        colorid = self.COLOR_TO_IDX[closest_color_name]
                        print(closest_color_name)

                        # Create o3d point cloud
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

                        labels = np.array(filt_pcd.cluster_dbscan(eps=0.05,min_points=10, print_progress=False))

                        unique_labels, counts = np.unique(labels[labels !=-1], return_counts=True)

                        # Only considering point clouds with at least one cluster (if not it means that all points are sparse and considered as noise)
                        if counts.size > 0:
                            # Append the detected object (class, color and position in the 2D map plane)

                            largest_cluster_label = unique_labels[np.argmax(counts)]
                            object_indices = np.where(labels == largest_cluster_label)[0]
                            filtered_pcd = filt_pcd.select_by_index(object_indices)

                            filtered_pcd.translate((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                            R = o3d.geometry.get_rotation_matrix_from_quaternion((tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z))
                            filtered_pcd.rotate(R, center=(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                            puntos = np.asarray(filtered_pcd.points)
                            centroid = puntos.mean(axis=0)

                            category = self.category_dict.get(int(classes[i].item()), 'None')
                            objetos.append(category)
                            
                            obj = DetectedObject()
                            obj.cls = category
                            obj.trackid = colorid
                            obj.pos2d = ROSPolygon(points=
                                [Point32(x=centroid[0], y=centroid[1], z=0.0),
                            ])
                            detected_objects.objects.append(obj)

                    except Exception as e:
                        print(f"ERROR: {type(e).__name__} - {e}")
                #print(len(objetos))
                if objetos:
                    # Publish all detected objects together
                    self.pub.publish(detected_objects)
                    print('PUBLISHING OBJECTS...')

    def camera(self):
        if np.array(self.pc).any() and np.array(self.color_image).any():

            from_frame_rel = self.target_frame
            to_frame_rel = self.map_frame

            try:
                tf = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
                print("TF")
            except TransformException as ex:
                self.get_logger().info('Could not transform frames.')
                return

            # YOLO application - specify the object classes to be detected in 'classes'. If the parameter is not included, all classes are considered. 
            results_bottle = self.model.track(self.color_image, persist=True, device="cpu", classes=[39], show=False, conf=0.5) #classes=[24,25,26,27,28,32,39,40,41,42,43,44,45,46,47,48,49,56,57,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79])
            results_ball = self.model.track(self.color_image, persist=True, device="cpu", classes=[32], show=False, conf=0.3)
            #results_ball = self.model.track(self.color_image, persist=True, device="cpu", classes=[39,32], show=True, conf=0.)
            
            self.detect_and_publish(results_bottle)
            self.detect_and_publish(results_ball)

            # Point cloud cropping using detected masks
            # if results[0].masks is not None:
            #     print(results[0].probs)
            #     detected_objects = DetectedObjectsList()
            #     objetos = []

            #     _, width = self.color_image.shape[:2]
            #     masks = results[0].masks.xy 
            #     classes = results[0].boxes.cls
            #     track_ids = results[0].boxes.id

            #     for i, mask in enumerate(masks):
            #         # Selecting points corresponding to object i
            #         binary_mask = np.zeros(self.color_image.shape[:2], dtype=np.uint8)
            #         cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
            #         indices = np.argwhere(binary_mask == 1)
            #         indices_1d = indices[:, 0] * width + indices[:, 1]
            #         cropped_vtx = self.pc[indices_1d[::10]]
            #         pcmask = (cropped_vtx[:,0] != 0) | (cropped_vtx[:,1] != 0) | (cropped_vtx[:,2] != 0) # Removing points placed in origin
            #         cropped_vtx = cropped_vtx[pcmask,:]

            #         # Only considering point clouds with more than 10 points - if not open3d functions tend to fail
            #         if len(cropped_vtx)>10:
            #             try:
            #                 # Selecting the closest color using the pixel values inside the object crop
            #                 selected_colors = self.color_image[indices[:, 0], indices[:, 1]]
            #                 avg_color = np.mean(selected_colors, axis=0)
            #                 avg_color=np.flip(avg_color)
            #                 distances = np.linalg.norm(colors_array - avg_color, axis=1)
            #                 closest_color_name = list(colors.keys())[np.argmin(distances)]
            #                 colorid = self.COLOR_TO_IDX[closest_color_name]
            #                 print(closest_color_name)

            #                 # Create o3d point cloud
            #                 cloudo3d = o3d.geometry.PointCloud()
            #                 pcd_points = np.array([(point[0], point[1], point[2]) for point in cropped_vtx], dtype=np.float32)
            #                 cloudo3d.points = o3d.utility.Vector3dVector(pcd_points)

            #                 # Filter point cloud
            #                 cloudo3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05,max_nn=30))
            #                 cloudo3d.orient_normals_consistent_tangent_plane(100)
            #                 normal_threshold = 0.4
            #                 normals = np.asarray(cloudo3d.normals)
            #                 indices = np.where(np.abs(normals[:,2])>normal_threshold)[0]
            #                 filt_pcd = cloudo3d.select_by_index(indices)

            #                 labels = np.array(filt_pcd.cluster_dbscan(eps=0.05,min_points=10, print_progress=False))

            #                 unique_labels, counts = np.unique(labels[labels !=-1], return_counts=True)

            #                 # Only considering point clouds with at least one cluster (if not it means that all points are sparse and considered as noise)
            #                 if counts.size > 0:
            #                     # Append the detected object (class, color and position in the 2D map plane)

            #                     largest_cluster_label = unique_labels[np.argmax(counts)]
            #                     object_indices = np.where(labels == largest_cluster_label)[0]
            #                     filtered_pcd = filt_pcd.select_by_index(object_indices)

            #                     filtered_pcd.translate((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
            #                     R = o3d.geometry.get_rotation_matrix_from_quaternion((tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z))
            #                     filtered_pcd.rotate(R, center=(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
            #                     puntos = np.asarray(filtered_pcd.points)
            #                     centroid = puntos.mean(axis=0)

            #                     category = self.category_dict.get(int(classes[i].item()), 'None')
            #                     objetos.append(category)
                                
            #                     obj = DetectedObject()
            #                     obj.cls = category
            #                     obj.trackid = colorid
            #                     obj.pos2d = ROSPolygon(points=
            #                         [Point32(x=centroid[0], y=centroid[1], z=0.0),
            #                     ])
            #                     detected_objects.objects.append(obj)

            #             except Exception as e:
            #                 print(f"ERROR: {type(e).__name__} - {e}")
            #     #print(len(objetos))
            #     if objetos:
            #         # Publish all detected objects together
            #         self.pub.publish(detected_objects)
            #         print('PUBLISHING OBJECTS...')

        self.color_image = []
        self.pc = []

def main(args=None):
    rclpy.init(args=args)
    realsense = Realsense()

    try:
        rclpy.spin(realsense)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            realsense.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()