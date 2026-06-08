#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
category_dict = {cat['id']: cat['color'] for cat in COCO_CATEGORIES}
category_dict2 = {cat['id']: cat['name'] for cat in COCO_CATEGORIES}

class Realsense(Node):

    def __init__(self):
        super().__init__('realsense')

        # Point cloud publisher
        self.pub = self.create_publisher(PointCloud2, 'topic', 10)

        # Point cloud message setup
        self.pc_msg = PointCloud2()
        self.header = Header()
        self.header.frame_id = 'prisma_rover/map' #'rover/map'
        self.pc_msg.height = 1
        self.pc_msg.is_dense = True
        self.pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.pc_msg.point_step = 15
        self.pc_msg.is_bigendian = False
        
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
        self.target_frame = self.declare_parameter('target_frame','prisma_rover/camera_rgb_optical_frame').get_parameter_value().string_value
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

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.05, self.camera)
        self.color_image = []
        self.pc = []

    def calculate_iou(self, box1, box2):

        poly1 = Polygon(box1)
        poly2 = Polygon(box2)

        intersection_area = poly1.intersection(poly2).area
        
        if intersection_area == 0:
            return 0.0
        return max(intersection_area/poly1.area, intersection_area/poly2.area)

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
        if np.array(self.pc).any() and np.array(self.color_image).any():

            from_frame_rel = self.target_frame
            to_frame_rel = 'prisma_rover/map'

            try:
                tf = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info('Could not transform frames.')
                return

            results = self.model.track(self.color_image, persist=True, device="cpu", classes=[24,25,26,27,28,32,39,40,41,42,43,44,45,46,47,48,49,56,57,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79])

            # Point cloud cropping using detected masks
            objetos = []
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
                            obb = filtered_pcd.get_oriented_bounding_box()
                            vertices = obb.get_box_points()
                            vertices = np.asarray(vertices)
                            
                            # Extracting geometric characteristics for spatial relationships: 2d obb, xmin, xmax, ymin, ymax, zmin, zmax
                            vertices2d = np.array([[x, z] for x, y, z in vertices])
                            ca = np.cov(vertices2d,y = None,rowvar = 0,bias = 1)
                            v, vect = np.linalg.eig(ca)
                            tvect = np.transpose(vect)
                            ar = np.dot(vertices2d,np.linalg.inv(tvect))
                            mina = np.min(ar,axis=0)
                            maxa = np.max(ar,axis=0)
                            diff = (maxa - mina)*0.5
                            center = mina + diff
                            corners = np.array([center+[-diff[0],-diff[1]],center+[diff[0],-diff[1]],center+[diff[0],diff[1]],center+[-diff[0],diff[1]],center+[-diff[0],-diff[1]]])
                            corners = np.dot(corners,tvect)
                            corners = corners[0:4]

                            category = category_dict2.get(int(classes[i].item()), 'None')
                            vertices = np.array(vertices)
                            objetos.append([corners,np.min(vertices[:, 0]),np.max(vertices[:, 0]),np.min(vertices[:, 1]),np.max(vertices[:, 1]),np.min(vertices[:, 2]),np.max(vertices[:, 2]), category])

                            # Color each point cloud according to its object class
                            color = category_dict.get(int(classes[i].item()), [0, 0, 0]) + [255]
                            points = [
                                Point(x=float(x) , y=float(y), z=float(z)) for x, y, z in filtered_pcd.points
                            ]
                            for j, point in enumerate(points):
                                r, g, b, a = color
                                buffer.extend(point_struct.pack(point.x, point.y, point.z, b, g, r, a))
                        except:
                            continue

                if objetos:
                    self.header.stamp = self.get_clock().now().to_msg()
                    self.pc_msg.header = self.header
                    self.pc_msg.width = len(buffer) // point_struct.size
                    self.pc_msg.point_step=point_struct.size
                    self.pc_msg.row_step = len(buffer)
                    self.pc_msg.data = buffer
                    print('Publishing...')
                    self.pub.publish(self.pc_msg)

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
