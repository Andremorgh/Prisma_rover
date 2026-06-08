#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Key mapping to move the robot
moveBindings = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Counter-clockwise rotation (left)
    'd': (0.0, -1.0),   # Clockwise rotation (right)
    'q': (1.0, 1.0),    # Forward-Left
    'e': (1.0, -1.0),   # Forward-Right
    'z': (-1.0, -1.0),  # Backward-Left
    'c': (-1.0, 1.0),   # Backward-Right
}

# Key mapping to modify speed limits
speedBindings = {
    'u': (1.1, 1.1),    # Increase linear and angular speed by 10%
    'i': (1.1, 1.0),    # Increase linear speed by 10%
    'o': (1.0, 1.1),    # Increase angular speed by 10%
    'j': (0.9, 0.9),    # Decrease linear and angular speed by 10%
    'k': (0.9, 1.0),    # Decrease linear speed by 10%
    'l': (1.0, 0.9),    # Decrease angular speed by 10%
}

msg = """
======================================================
     PRISMA ROVER / GENERIC TELEOP KEYBOARD NODE      
======================================================
Movement controls:
        q    w    e
        a    s    d
        z    x    c

w/s : forward/backward
a/d : rotation left/right
q/e : forward curve left/right
z/c : backward curve left/right
space or x : stop

Speed limit adjustments:
u/j : increase/decrease max speed (linear & angular) by 10%
i/k : increase/decrease max linear speed only by 10%
o/l : increase/decrease max angular speed only by 10%

CTRL-C to exit
======================================================
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Declare node parameters for maximum reusability
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('max_linear_vel', 0.5)    # m/s
        self.declare_parameter('max_angular_vel', 1.0)   # rad/s
        self.declare_parameter('linear_step', 0.05)      # linear increment step
        self.declare_parameter('angular_step', 0.1)      # angular increment step
        
        # Parameter retrieval
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.speed = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.turn = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.speed_step = self.get_parameter('linear_step').get_parameter_value().double_value
        self.turn_step = self.get_parameter('angular_step').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        self.get_logger().info(f"Teleop started on topic: '{self.cmd_vel_topic}'")
        self.get_logger().info(f"Initial limits: linear={self.speed:.2f} m/s, angular={self.turn:.2f} rad/s")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        
        target_linear = 0.0
        target_angular = 0.0
        control_linear = 0.0
        control_angular = 0.0
        
        print(msg)
        
        try:
            while True:
                key = self.getKey(settings)
                
                if key in moveBindings.keys():
                    x, th = moveBindings[key]
                    target_linear = x * self.speed
                    target_angular = th * self.turn
                elif key in [' ', 'x']:
                    target_linear = 0.0
                    target_angular = 0.0
                elif key in speedBindings.keys():
                    factor_speed, factor_turn = speedBindings[key]
                    self.speed = self.speed * factor_speed
                    self.turn = self.turn * factor_turn
                    # Show updated limits on screen
                    sys.stdout.write(f"\rLimits updated -> linear: {self.speed:.2f} m/s | angular: {self.turn:.2f} rad/s        ")
                    sys.stdout.flush()
                    continue
                else:
                    # If no valid key is pressed within the timeout, decay speed or hold
                    if (key == ''):
                        pass
                    if (key == '\x03'):  # CTRL-C
                        break
                
                # Simple ramp profile (smoothing) to avoid sudden accelerations
                control_linear = target_linear
                control_angular = target_angular
                
                # Publish the message
                twist = Twist()
                twist.linear.x = float(control_linear)
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = float(control_angular)
                
                self.publisher_.publish(twist)
                
                # Real-time terminal feedback (overwriting current line)
                sys.stdout.write(f"\rCurrent Velocity -> linear: {control_linear:+.2f} m/s | angular: {control_angular:+.2f} rad/s        ")
                sys.stdout.flush()
                
        except Exception as e:
            self.get_logger().error(f"Error during teleop execution: {e}")
            
        finally:
            # On exit, force the robot to stop and restore terminal settings
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nTeleoperation stopped. Robot state set to stopped.")
 
    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        # Timeout at 0.1 seconds to allow responsive inputs and spin
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
