import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64MultiArray
from neck_cmds.servo_motor_neck import *
# bug here

class neckSubscriber(Node):

    def __init__(self):
        super().__init__('neck_node')
        self.get_logger().info("start neck")
        self.i = 0
        self.sub_neck = self.create_subscription(
            Float64MultiArray, 
            '/position', 
            self.listener_callback,
            10)
        self.sub_neck # prevent unused variable warning
        self.ctime = time.time()

    def listener_callback(self, msg):
        new_time = time.time()
        fps = 1. / (new_time - self.ctime)
        self.ctime = new_time
        self.get_logger().info('neck cmds:"%s" fps: "%s"' % (len(msg.data), int(fps)))


def main(args=None):
    rclpy.init(args=args)

    neck_subscriber = neckSubscriber()

    rclpy.spin(neck_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    neck_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()