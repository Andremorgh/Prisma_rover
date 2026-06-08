import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        
        # Parameters
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('map_frame', 'prisma_rover/map')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PoseStamped, goal_topic, 10)
        self.get_logger().info(f"Targeting goal topic: {goal_topic} in frame: {self.map_frame}")

    def publish_goal(self, x, y):
        msg = PoseStamped()
        
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0 
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Goal published -> X: {x:.2f}, Y: {y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()

    try:
        print("--- Send a Goal to the Rover (prisma_rover_quantum_controller) ---")
        user_input = input("Enter coordinates (x, y): ") 
        
        if ',' in user_input:
            parts = user_input.split(',')
            x_val = float(parts[0].strip())
            y_val = float(parts[1].strip())
            
            node.publish_goal(x_val, y_val)
            
            # Wait for message to flush
            time.sleep(1.0) 
        else:
            print("Error: Format must be 'x, y' (e.g., 2.5, 3.0)")

    except ValueError:
        print("Error: The inputs must be valid numbers.")
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()