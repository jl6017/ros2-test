import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from .servo_motor_face import *

class mouthSubscriber(Node):

    def __init__(self):
        super().__init__('mouth_node')
        self.get_logger().info("start mouth")
        self.i = 0
        self.sub_mouth = self.create_subscription(
            Float64MultiArray, 
            '/mouth_cmds', 
            self.listener_callback,
            10)
        self.sub_mouth  # prevent unused variable warning
        self.ctime = time.time()
        self.data_record = [0.5] * 7

    def listener_callback(self, msg):
        new_time = time.time()
        fps = 1. / (new_time - self.ctime)
        self.ctime = new_time
        # self.get_logger().info('mouth cmds:"%s" fps: "%s"' % (msg.data, int(fps)))

        dist = [abs(self.data_record[i] - msg.data[i]) for i in range(len(msg.data))]
        avg_dis = np.mean(dist)
        if avg_dis > 0.2:
            move_mouth(msg.data)
            self.data_record = msg.data
        


def main(args=None):
    rclpy.init(args=args)

    mouth_subscriber = mouthSubscriber()

    rclpy.spin(mouth_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mouth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()