#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge_Twist')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'tricycle_steering_controller/reference_unstamped', 10)
        
    def cmd_vel_callback(self, msg):
        twist_msg = Twist()
        twist_msg = msg
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_bridge = CmdVelBridge()
    rclpy.spin(cmd_vel_bridge)
    cmd_vel_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
