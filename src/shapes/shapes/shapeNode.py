#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(Int32, 'shape', 10)
        self.timer = self.create_timer(2, self.user_input)

    def user_input(self):
        shape =int(input("\n\n\n Enter shape number: \n 1. cornu curve (aka euler curve) \n 2. deltoid curve (NOT A TRIANGLE)" \
        "\n 3. seashell spiral (logarithmic spiral) \n 4. sparkle star (astroid curve) \n\n\n enter the number: "))
        msg = Int32()
        msg.data = shape
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
