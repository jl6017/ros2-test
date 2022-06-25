import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64MultiArray
from .servo_motor_face import *

class eyeSubscriber(Node):

    def __init__(self):
        super().__init__('eye_node')
        self.get_logger().info("start eye")
        self.i = 0
        self.sub_eye = self.create_subscription(
            Float64MultiArray, 
            '/eye_cmds', 
            self.listener_callback,
            10)
        self.sub_eye  # prevent unused variable warning
        self.ctime = time.time()

    def listener_callback(self, msg):
        new_time = time.time()
        fps = 1. / (new_time - self.ctime)
        self.ctime = new_time        
        self.get_logger().info('eye cmds:"%s" fps: "%s"' % (len(msg.data), int(fps)))


def main(args=None):
    slowly_open()
    rclpy.init(args=args)

    eye_subscriber = eyeSubscriber()

    rclpy.spin(eye_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    eye_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()