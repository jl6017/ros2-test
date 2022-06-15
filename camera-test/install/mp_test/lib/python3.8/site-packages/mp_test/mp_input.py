import rclpy
from rclpy.node import Node

class mpNode(Node):
    """
    mediapipe input node
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("This is %s!" % name)

def main(args=None):

    rclpy.init(args=args)
    node = mpNode("mediapipe")
    rclpy.spin(node)
    rclpy.shutdown()