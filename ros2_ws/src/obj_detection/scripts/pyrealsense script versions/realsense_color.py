#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2
from ultralytics import YOLO
from shapely.geometry import Polygon
from sklearn.cluster import DBSCAN
import time
import struct

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection

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
        self.header.frame_id = 'camera'
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

        # YOLO setup
        self.model = YOLO("/home/user/ros2_ws/src/obj_detection/scripts/yolo11n-seg.pt")
        self.pos = [0,0]
        self.ori = [0,0,0,1]
        self.transformation_matrix = np.array([
            [1,  0,  0],  # x2 =  x1
            [0,  0,  1],  # y2 =  z1
            [0, -1,  0]   # z2 = -y1
        ])

        self.camera()

    def calculate_iou(self, box1, box2):

        poly1 = Polygon(box1)
        poly2 = Polygon(box2)

        # ---------------- PROVISIONAL!! PLOTEANDO INTERSECCIONES ----------------------------------------

        '''fig, ax = plt.subplots()

        # Crear un parche (patch) de Matplotlib a partir de los polígonos de Shapely
        patches = []

        # Convertir el polígono de Shapely a un parche y añadirlo a la lista
        patch1 = MplPolygon(list(poly1.exterior.coords), closed=True, edgecolor='blue', facecolor='lightblue', alpha=0.5)
        patch2 = MplPolygon(list(poly2.exterior.coords), closed=True, edgecolor='red', facecolor='lightcoral', alpha=0.5)

        patches.append(patch1)
        patches.append(patch2)

        # Añadir los parches al gráfico
        ax.add_patch(patch1)
        ax.add_patch(patch2)

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])

        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()'''
        # ------------------------------------------------------------------------------------------------

        intersection_area = poly1.intersection(poly2).area
        
        if intersection_area == 0:
            return 0.0
        return max(intersection_area/poly1.area, intersection_area/poly2.area)

    def camera(self):
        while rclpy.ok():
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
            objetos = []
            if results[0].masks is not None:
                height, width = color_image.shape[:2]
                masks = results[0].masks.xy
                classes = results[0].boxes.cls

                buffer = bytearray()
                point_struct = struct.Struct("<fffBBBB")

                for i, mask in enumerate(masks):
                    # Selecting points corresponding to object i
                    binary_mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
                    kernel = np.ones((13, 13), np.uint8)
                    binary_mask = cv2.erode(binary_mask, kernel, iterations=1)
                    indices = np.argwhere(binary_mask == 1)
                    indices_1d = indices[:, 0] * width + indices[:, 1]
                    #cropped_vtx = vtx[indices_1d]
                    cropped_vtx = vtx[indices_1d[::5]]
                    pcmask = (cropped_vtx['f0'] != 0) | (cropped_vtx['f1'] != 0) | (cropped_vtx['f2'] != 0) # Removing points placed in origin
                    cropped_vtx = cropped_vtx[pcmask]

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
                            # Create o3d point cloud for 3d OBB
                            cloudo3d = o3d.geometry.PointCloud()
                            pcd_points = np.array([(point['f0'], point['f1'], point['f2']) for point in cropped_vtx], dtype=np.float32)
                            cloudo3d.points = o3d.utility.Vector3dVector(pcd_points)
                            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
                            R = mesh.get_rotation_matrix_from_quaternion((self.ori[1],self.ori[2],self.ori[3],self.ori[0]))
                            cloudo3d.rotate(R, center = (0,0,0))
                            cloudo3d.translate((self.pos[0],self.pos[1],0))

                            obb = cloudo3d.get_oriented_bounding_box()
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
                            
                            cropped_vtx = np.array([(point['f0'], point['f1'], point['f2']) for point in cropped_vtx[::5]], dtype=np.float32)

                            # Color each point cloud according to its object class
                            color = category_dict.get(int(classes[i].item()), [0, 0, 0]) + [255]
                            points = [
                                Point(x=float(x) , y=float(y), z=float(z)) for x, y, z in cropped_vtx
                            ]
                            for j, point in enumerate(points):
                                r, g, b, a = color
                                buffer.extend(point_struct.pack(point.x, point.y, point.z, b, g, r, a))
                        except:
                            continue

                if objetos:
                    # Extracting object relationships (support, adjacency - PARENT MISSING)
                    # Checking object proximity and overlapping
                    boxes = [item[0] for item in objetos]
                    iou_pairs = []
                    nextto_pairs = []
                    for i in range(len(boxes)):
                        for j in range(i + 1, len(boxes)):
                            iou = self.calculate_iou(boxes[i], boxes[j])
                            if iou > 0.6:
                                #print(f"Object {i} ({objetos[i][7]}) and {j} ({objetos[j][7]}) overlap: {iou}")
                                iou_pairs.append((i, j))

                            # Adjacency
                            x_interseccion = (objetos[i][2] >= objetos[j][1] - 0.05) and (objetos[i][1] <= objetos[j][2] + 0.05)
                            y_interseccion = (objetos[i][4] >= objetos[j][3] - 0.05) and (objetos[i][3] <= objetos[j][4] + 0.05)
                            z_interseccion = (objetos[i][6] >= objetos[j][5] - 0.05) and (objetos[i][5] <= objetos[j][6] + 0.05)
                            if x_interseccion and y_interseccion and z_interseccion:
                                nextto_pairs.append((i, j))

                    # Checking distance between cube faces for support relationship
                    supported_pairs=[]
                    for pair in iou_pairs:
                        #print(f"Object {pair[0]} ({objetos[pair[0]][7]}) and {pair[1]} ({objetos[pair[1]][7]}): {abs(objetos[pair[0]][3] - objetos[pair[1]][4])}")
                        #print(f"Object {pair[0]} ({objetos[pair[0]][7]}) and {pair[1]} ({objetos[pair[1]][7]}): {abs(objetos[pair[0]][4] - objetos[pair[1]][3])}")
                        if abs(objetos[pair[0]][3] - objetos[pair[1]][4])<0.1 and objetos[pair[1]][3]<objetos[pair[0]][3]: #Objeto 1 sobre objeto 0
                            supported_pairs.append((pair[1],pair[0]))
                        if abs(objetos[pair[0]][4] - objetos[pair[1]][3])<0.1 and objetos[pair[0]][3]<objetos[pair[1]][3]: #Objeto 0 sobre objeto 1
                            supported_pairs.append((pair[0],pair[1]))

                    # Removing repeated relationships (support prevails over adjacency)
                    supported_sets = [set(pair) for pair in supported_pairs]
                    nextto_pairs = [elem for elem in nextto_pairs if set(elem) not in supported_sets]
                    #nextto_pairs = [elem for elem in nextto_pairs if elem not in supported_pairs]

                    for pair in supported_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) ON object {pair[1]} ({objetos[pair[1]][7]})")
                    for pair in nextto_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) and object {pair[1]} ({objetos[pair[1]][7]}) are nearby")

                    self.header.stamp = self.get_clock().now().to_msg()
                    self.pc_msg.header = self.header
                    self.pc_msg.width = len(buffer) // point_struct.size
                    self.pc_msg.point_step=point_struct.size
                    self.pc_msg.row_step = len(buffer)
                    self.pc_msg.data = buffer
                    print('Publicando...')
                    self.pub.publish(self.pc_msg)

def main(args=None):
    rclpy.init(args=args)

    realsense = Realsense()
    rclpy.spin(realsense)

    realsense.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
