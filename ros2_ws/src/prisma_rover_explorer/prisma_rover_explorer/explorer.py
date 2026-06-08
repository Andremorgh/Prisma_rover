import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from collections import deque
import math


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Declare parameters
        self.declare_parameter('map_topic', '/prisma_rover/global_costmap/costmap')
        self.declare_parameter('pose_topic', '/prisma_rover/pose')
        self.declare_parameter('navigate_to_pose_action', '/prisma_rover/navigate_to_pose')
        self.declare_parameter('map_frame', 'prisma_rover/map')
        self.declare_parameter('sector_size', 4.0)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('navigate_to_pose_action').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.sector_size = self.get_parameter('sector_size').get_parameter_value().double_value

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10)

        # Subscriber to robot position (amcl_pose or similar)
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, nav_action)

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # (x, y) in world coordinates
        self.robot_orientation = 0.0

        # Sector exploration parameters
        self.sectors = {}  # Dictionary to track sector exploration status
        self.sector_queue = deque()  # Queue of sectors to explore
        self.current_goal_sector = None
        self.current_goal_id = None
        self.sectors_initialized = False  # Flag to track if sectors are already initialized

        # Timer for periodic exploration
        self.timer = self.create_timer(5.0, self.explore_sectors)
        
        # Flag to check if navigation is in progress
        self.navigation_in_progress = False

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received", throttle_duration_sec=10.0)  # Throttle log messages
        self.update_sectors()

    def pose_callback(self, msg):
        # Update robot position from localization
        self.robot_position = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        # Extract yaw from quaternion
        orientation = msg.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def update_sectors(self):
        """Dynamically divide the map into sectors based on a global anchor grid"""
        if self.map_data is None:
            return

        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        width = self.map_data.info.width
        height = self.map_data.info.height

        min_x = origin_x
        max_x = origin_x + width * resolution
        min_y = origin_y
        max_y = origin_y + height * resolution

        start_i = int(math.floor(min_x / self.sector_size))
        end_i = int(math.ceil(max_x / self.sector_size))
        start_j = int(math.floor(min_y / self.sector_size))
        end_j = int(math.ceil(max_y / self.sector_size))

        added_new = 0
        for i in range(start_i, end_i):
            for j in range(start_j, end_j):
                sector_id = f"{i}_{j}"
                
                # If already discovered, skip
                if sector_id in self.sectors:
                    continue

                # Calculate sector center in world coordinates anchored to (0.0, 0.0)
                sector_center_x = (i + 0.5) * self.sector_size
                sector_center_y = (j + 0.5) * self.sector_size
                
                # Check if sector center is within map bounds and is free space
                if self.is_valid_sector(sector_center_x, sector_center_y):
                    self.sectors[sector_id] = {
                        'center_x': sector_center_x,
                        'center_y': sector_center_y,
                        'explored': False,
                        'coordinates': (i, j)
                    }
                    added_new += 1

        if added_new > 0:
            self.get_logger().info(f"Discovered {added_new} new sectors. Total active unexplored/explored: {len(self.sectors)}")

    def is_valid_sector(self, x, y):
        """Check if sector center is valid (not occupied and within map)"""
        if self.map_data is None:
            return False

        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check if within map bounds
        if (map_x < 0 or map_x >= self.map_data.info.width or 
            map_y < 0 or map_y >= self.map_data.info.height):
            return False

        # Check if not occupied (0 = free, -1 = unknown, >0 = occupied)
        map_index = map_y * self.map_data.info.width + map_x
        if (map_index < len(self.map_data.data) and 
            self.map_data.data[map_index] > 50):  # Threshold for occupied
            return False

        return True

    def get_closest_unexplored_sector(self):
        """Find the closest unexplored sector to the robot"""
        closest_sector = None
        min_distance = float('inf')

        for sector_id, sector_data in self.sectors.items():
            if not sector_data['explored']:
                distance = math.sqrt(
                    (self.robot_position[0] - sector_data['center_x'])**2 +
                    (self.robot_position[1] - sector_data['center_y'])**2
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_sector = sector_id

        return closest_sector

    def navigate_to(self, x, y):
        """Send navigation goal to Nav2"""
        if self.navigation_in_progress:
            self.get_logger().info("Navigation already in progress, skipping new goal")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to sector center: x={x:.2f}, y={y:.2f}")

        # Wait for the action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warning("Navigation server not available")
            return

        # Send the goal and register a callback for the result
        self.navigation_in_progress = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.navigation_in_progress = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """Callback to handle the result of the navigation action"""
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
            
            # Mark current sector as explored
            if self.current_goal_sector:
                self.sectors[self.current_goal_sector]['explored'] = True
                self.get_logger().info(f"Sector {self.current_goal_sector} marked as explored")
            
            self.navigation_in_progress = False
            self.current_goal_sector = None

        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            self.navigation_in_progress = False
            self.current_goal_sector = None

    def explore_sectors(self):
        """Explore sectors periodically"""
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if self.navigation_in_progress:
            self.get_logger().info("Navigation in progress, skipping exploration cycle")
            return

        # Find closest unexplored sector
        closest_sector = self.get_closest_unexplored_sector()

        if not closest_sector:
            self.get_logger().info("All sectors explored! Exploration complete!")
            return

        # Get sector data
        sector_data = self.sectors[closest_sector]
        
        # Navigate to sector center
        self.current_goal_sector = closest_sector
        self.navigate_to(sector_data['center_x'], sector_data['center_y'])

    def is_robot_in_sector(self, sector_id):
        """Check if robot is currently in the specified sector"""
        if sector_id not in self.sectors:
            return False
            
        sector_data = self.sectors[sector_id]
        sector_min_x = sector_data['center_x'] - self.sector_size / 2
        sector_max_x = sector_data['center_x'] + self.sector_size / 2
        sector_min_y = sector_data['center_y'] - self.sector_size / 2
        sector_max_y = sector_data['center_y'] + self.sector_size / 2
        
        return (sector_min_x <= self.robot_position[0] <= sector_max_x and
                sector_min_y <= self.robot_position[1] <= sector_max_y)


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting sector-based exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        if rclpy.ok():
            explorer_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()