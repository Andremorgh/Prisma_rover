#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from prisma_rover_interfaces.msg import DetectedObjectsList
from shapely.geometry import Polygon
import json
import atexit
import numpy as np

class ObjectsMap(Node):

    def __init__(self):
        super().__init__('objects_map')

        self.sub = self.create_subscription(DetectedObjectsList,'objects_detected',self.map_callback,10)
        self.declare_parameter('dataset_path', '/home/user/ros2_ws/src/obj_detection/scripts/dataset')
        self.dataset_path = self.get_parameter('dataset_path').get_parameter_value().string_value
        self.memoryIDs = []
        self.stored_objects = []
        self.correspondences= {}
        atexit.register(self.save_to_json)

    def calculate_iou(self, box1, box2):
        poly1 = Polygon(box1)
        poly2 = Polygon(box2)

        intersection_area = poly1.intersection(poly2).area
        union_area = poly1.union(poly2).area
        if intersection_area == 0:
            return 0.0
        return intersection_area / union_area

    def serialize_object(self, obj):
        return {
            'cls': str(obj.cls),
            'trackID': int(obj.trackid),
            'pos2D': [{'x': float(p.x), 'y': float(p.y), 'z': float(p.z)} for p in obj.pos2d.points],
            'minh': float(obj.minh),
            'maxh': float(obj.maxh),
            'relON': int(obj.rel_on),
            'relUNDER': [int(i) for i in obj.rel_under],
            'relNEARBY': [int(i) for i in obj.rel_nearby],
        }

    def save_to_json(self):

        import os
        filename = os.path.join(self.dataset_path, 'stored_objects.json')
        with open(filename, 'w') as f:
            json.dump(self.stored_objects, f, indent=4)
        self.get_logger().info(f'Data stored in {filename}')

    def map_callback(self, msg):

        print('--------------------------------------------------------------------')
        self.get_logger().info(f'Message received with {len(msg.objects)} objects')

        # STEP 1: SAVING CORRESPONDENCES
        for i, obj in enumerate(msg.objects):
            if obj.trackid not in self.memoryIDs:
                self.memoryIDs.append(obj.trackid)

                # Check if there are previous IDs with the same class
                selected_objects = [element for element in self.stored_objects if element['cls'] == obj.cls]

                if selected_objects:
                    # Check if the selected objects are in the same spot
                    obj_points = [{'x': p.x, 'y': p.y, 'z': p.z} for p in obj.pos2d.points]
                    obj_points = [(point['x'], point['y']) for point in obj_points]

                    ious = []
                    for element in selected_objects:
                        points = [(point['x'], point['y']) for point in element['pos2D']]
                        iou = self.calculate_iou(obj_points, points)
                        ious.append(iou)

                    if all(valor < 0.0 for valor in ious):
                        # new object since there are no other objects in the same spot
                        #self.get_logger().info(f'New object with class {obj.cls} and ID {obj.trackid}')
                        self.correspondences[obj.trackid] = []
                        self.stored_objects.append(self.serialize_object(obj))
                    else:
                        #self.get_logger().info(f'Object with ID {obj.trackid} already registered with another ID value.')
                        max_iou = max(enumerate(ious), key=lambda x: x[1])[0]
                        self.correspondences[selected_objects[max_iou]['trackID']].append(obj.trackid)
                else:
                    # New object since there are no other objects with the same class
                    #self.get_logger().info(f'New object with class {obj.cls} and ID {obj.trackid}')
                    self.correspondences[obj.trackid] = []
                    self.stored_objects.append(self.serialize_object(obj))

        # STEP 2: UPDATING OBJECTS IN GLOBAL MAP
        for i, obj in enumerate(msg.objects):

            trackid = obj.trackid
            equivalence = next((k for k, v in self.correspondences.items() if trackid in v), None)
            if equivalence:
                trackid = equivalence

            # Look for object ID in stored objects
            stored_obj_idx = next((i for i, ob in enumerate(self.stored_objects) if ob['trackID'] == trackid), None)

            # Update relNEARBY
            obj_ne = [int(i) for i in obj.rel_nearby if i is not self.stored_objects[stored_obj_idx]['trackID']] # New relationships
            near = obj_ne + self.stored_objects[stored_obj_idx]['relNEARBY'] # Add new to old
            for j, element in enumerate(near):
                equiv = next((k for k, v in self.correspondences.items() if element in v), None)
                if equiv:
                    near[j] = equiv
            self.stored_objects[stored_obj_idx]['relNEARBY'] = sorted(set(near))

            # Update relON and relUNDER
            relON = obj.rel_on
            if relON<65535:
                equiv = next((k for k, v in self.correspondences.items() if relON in v), None)
                if equiv:
                    relON = equiv

                if relON is not self.stored_objects[stored_obj_idx]['trackID']:
                    self.stored_objects[stored_obj_idx]['relON'] = relON
                    if relON in self.stored_objects[stored_obj_idx]['relNEARBY']:
                        self.stored_objects[stored_obj_idx]['relNEARBY'] = [j for j in self.stored_objects[stored_obj_idx]['relNEARBY'] if j is not relON]

                on_obj_idx = next((i for i, ob in enumerate(self.stored_objects) if ob['trackID'] == relON), None)
                if self.stored_objects[stored_obj_idx]['trackID'] not in self.stored_objects[on_obj_idx]['relUNDER'] and self.stored_objects[stored_obj_idx]['trackID'] is not self.stored_objects[on_obj_idx]['trackID']:
                    self.stored_objects[on_obj_idx]['relUNDER'].append(self.stored_objects[stored_obj_idx]['trackID'])
                    if self.stored_objects[stored_obj_idx]['trackID'] in self.stored_objects[on_obj_idx]['relNEARBY']:
                        self.stored_objects[on_obj_idx]['relNEARBY'] = [j for j in self.stored_objects[on_obj_idx]['relNEARBY'] if j is not self.stored_objects[stored_obj_idx]['trackID']]

        self.save_to_json()

def main(args=None):

    rclpy.init(args=args)
    objects_map = ObjectsMap()
    rclpy.spin(objects_map)

    objects_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
