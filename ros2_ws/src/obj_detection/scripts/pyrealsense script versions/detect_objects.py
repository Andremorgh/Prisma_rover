#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon as ROSPolygon
from geometry_msgs.msg import Point32
from obj_msgs.msg import DetectedObject, DetectedObjectsList

from shapely.geometry import Polygon
from sklearn.cluster import DBSCAN
from ultralytics import YOLO
import pyrealsense2 as rs
import open3d as o3d
import numpy as np
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

class Realsense(Node):

    def __init__(self):
        super().__init__('realsense')

        #TODO: SUBSCRIBE TO ROBOT POSE AND TRANSFORM POINT CLOUD

        self.pub = self.create_publisher(DetectedObjectsList, 'objects_detected', 10)
        
        # Realsense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60)
        self.pc = rs.pointcloud()
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.pipeline.start(self.config)
        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.min_distance, 0.1)
        self.threshold_filter.set_option(rs.option.max_distance, 1.5)
        self.transformation_matrix = np.array([
            [1,  0,  0],  # x2 =  x1
            [0,  0,  1],  # y2 =  z1
            [0, -1,  0]   # z2 = -y1
        ])

        # YOLO setup
        self.model = YOLO("/home/user/ros2_ws/src/obj_detection/scripts/yolo11n-seg.pt")
        #with open('/home/user/ros2_ws/src/obj_detection/scripts/stored_objects.json', 'r') as file:
        #    COCO_CATEGORIES = json.load(file)
        self.category_dict = {cat['id']: cat['name'] for cat in COCO_CATEGORIES}

        self.camera()

    def calculate_iou(self, box1, box2):

        poly1 = Polygon(box1)
        poly2 = Polygon(box2)
        intersection_area = poly1.intersection(poly2).area
        
        if intersection_area == 0:
            return 0.0

        return max(intersection_area/poly1.area, intersection_area/poly2.area)

    def camera(self):
        while rclpy.ok():
            start = time.time()
            # Obtain aligned RGB and depth images
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Extract point cloud from depth
            depth_frame = self.threshold_filter.process(depth_frame)
            self.pc.map_to(color_frame)
            points = self.pc.calculate(depth_frame)
            vtx = np.asanyarray(points.get_vertices())

            # Apply YOLO on color image
            color_image = np.asanyarray(color_frame.get_data())
            results = self.model.track(color_image, persist=True, device="cpu", classes=[24,25,26,27,28,32,39,40,41,42,43,44,45,46,47,48,49,56,57,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79])

            # Point cloud cropping using detected masks
            if results[0].boxes.id is not None:

                detected_objects = DetectedObjectsList()
                objetos = []

                _, width = color_image.shape[:2]
                masks = results[0].masks.xy
                classes = results[0].boxes.cls
                track_ids = results[0].boxes.id

                for i, mask in enumerate(masks):
                    # Selecting points corresponding to object i
                    binary_mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
                    kernel = np.ones((13, 13), np.uint8)
                    binary_mask = cv2.erode(binary_mask, kernel, iterations=1)
                    indices = np.argwhere(binary_mask == 1)
                    indices_1d = indices[:, 0] * width + indices[:, 1]
                    cropped_vtx = vtx[indices_1d[::5]]
                    pcmask = (cropped_vtx['f0'] != 0) | (cropped_vtx['f1'] != 0) | (cropped_vtx['f2'] != 0) # Removing points placed in origin (noise)
                    cropped_vtx = cropped_vtx[pcmask]

                    # The cropped point cloud is evaluated only if it has enough points
                    if len(cropped_vtx)>10:
                        # Filtering to remove outliers
                        pts2d = np.array([[x, z] for x, y, z in cropped_vtx])
                        dbscan = DBSCAN(eps=0.1, min_samples=20)
                        dbscan.fit(pts2d)
                        labels = dbscan.labels_
                        unique_labels = set(labels)
                        unique_labels.discard(-1)
                        if len(unique_labels)>0:
                            largest_cluster_label = max(unique_labels, key=lambda x: np.sum(labels == x))
                            largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
                            cropped_vtx = cropped_vtx[largest_cluster_indices]

                        try:
                            # Create o3d point cloud for 3d OBB - including axis rotation
                            cloudo3d = o3d.geometry.PointCloud()
                            pcd_points=np.array([(point['f0'], point['f1'], point['f2']) for point in cropped_vtx], dtype=np.float32)
                            cloudo3d.points = o3d.utility.Vector3dVector(np.dot(pcd_points, self.transformation_matrix.T))
                            obb = cloudo3d.get_oriented_bounding_box()
                            vertices = obb.get_box_points()
                            vertices = np.asarray(vertices)
                            
                            # Extracting geometric characteristics
                            # 2D oriented bounding box
                            vertices2d = np.array([[x, y] for x, y, z in vertices])
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

                            category = self.category_dict.get(int(classes[i].item()), 'None')
                            vertices = np.array(vertices)
                            objetos.append([corners,np.min(vertices[:, 0]),np.max(vertices[:, 0]),np.min(vertices[:, 1]),np.max(vertices[:, 1]),np.min(vertices[:, 2]),np.max(vertices[:, 2]),category])
                            
                            obj = DetectedObject()
                            obj.cls = category
                            obj.trackid = int(track_ids[i].item())
                            obj.pos2d = ROSPolygon(points=
                                [Point32(x=corners[0][0], y=corners[0][1], z=0.0),
                                Point32(x=corners[1][0], y=corners[1][1], z=0.0),
                                Point32(x=corners[2][0], y=corners[2][1], z=0.0),
                                Point32(x=corners[3][0], y=corners[3][1], z=0.0)
                            ])
                            obj.minh = np.min(vertices[:, 2])
                            obj.maxh = np.max(vertices[:, 2])
                            obj.rel_on = 65535 # Max. value if there is no relationship
                            obj.rel_under = []
                            obj.rel_nearby = []
                            detected_objects.objects.append(obj)

                        except Exception as e:
                            print(f"Ocurrió un error: {type(e).__name__} - {e}")

                if objetos:
                    # Extracting object relationships (support, adjacency)
                    boxes = [item[0] for item in objetos]
                    iou_pairs = []
                    nextto_pairs = []
                    for i in range(len(boxes)):
                        for j in range(i + 1, len(boxes)):
                            iou = self.calculate_iou(boxes[i], boxes[j])
                            if iou > 0.6:
                                iou_pairs.append((i, j))
                            # Adjacency relationship
                            x_interseccion = (objetos[i][2]>=objetos[j][1] and objetos[j][2]>=objetos[i][1]) or (objetos[j][1]-objetos[i][2]<=0.1) or (objetos[i][1]-objetos[j][2]<=0.1)
                            y_interseccion = (objetos[i][4]>=objetos[j][3] and objetos[j][4]>=objetos[i][3]) or (objetos[j][3]-objetos[i][4]<=0.1) or (objetos[i][3]-objetos[j][4]<=0.1)
                            z_interseccion = (objetos[i][6]>=objetos[j][5] and objetos[j][6]>=objetos[i][5]) or (objetos[j][5]-objetos[i][6]<=0.1) or (objetos[i][5]-objetos[j][6]<=0.1)
                            if x_interseccion and y_interseccion and z_interseccion:
                                nextto_pairs.append((i, j))

                    # Support relationship
                    supported_pairs=[]
                    for pair in iou_pairs:
                        if abs(objetos[pair[0]][6] - objetos[pair[1]][5])<0.1 and objetos[pair[0]][6]<objetos[pair[1]][6]: #Object 1 over object 0
                            supported_pairs.append((pair[1],pair[0]))
                        if abs(objetos[pair[0]][5] - objetos[pair[1]][6])<0.1 and objetos[pair[1]][6]<objetos[pair[0]][6]: #Object 0 over object 1
                            supported_pairs.append((pair[0],pair[1]))

                    # Removing repeated relationships (support prevails over adjacency)
                    supported_sets = [set(pair) for pair in supported_pairs]
                    nextto_pairs = [elem for elem in nextto_pairs if set(elem) not in supported_sets]

                    for pair in supported_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) ON object {pair[1]} ({objetos[pair[1]][7]})")
                        A, B = pair
                        detected_objects.objects[int(A)].rel_on=detected_objects.objects[int(B)].trackid
                        detected_objects.objects[int(B)].rel_under.append(detected_objects.objects[int(A)].trackid)

                    for pair in nextto_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) and object {pair[1]} ({objetos[pair[1]][7]}) are nearby")
                        A, B = pair
                        detected_objects.objects[int(A)].rel_nearby.append(detected_objects.objects[int(B)].trackid)
                        detected_objects.objects[int(B)].rel_nearby.append(detected_objects.objects[int(A)].trackid)

                    self.pub.publish(detected_objects)
                    print(1/(time.time()-start),' FPS')

def main(args=None):
    rclpy.init(args=args)

    realsense = Realsense()
    rclpy.spin(realsense)

    realsense.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
