import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray


class nnSubscriber(Node):

    def __init__(self):
        super().__init__('nn_node')
        self.i = 0
        self.sub_points = self.create_subscription(
            Float64MultiArray, 
            '/simple_lmks', 
            self.listener_callback,
            10)
        self.sub_points  # prevent unused variable warning
        self.pub_cmds = self.create_publisher(Float64MultiArray, '/cmds', 10)

    def listener_callback(self, msg):
        msg_out = Float64MultiArray()

        simple_lmks = np.array([msg.data[:113], msg.data[113:]])
        print(simple_lmks.shape)
        if self.i == 0:
            self.H_static_face = np.copy(simple_lmks)


        self.get_logger().info('nn get: "%s"' % len(msg.data))

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    nn_subscriber = nnSubscriber()

    rclpy.spin(nn_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nn_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()