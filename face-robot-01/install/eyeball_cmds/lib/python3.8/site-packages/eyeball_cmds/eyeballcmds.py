import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64MultiArray


class eyeballSubscriber(Node):

    def __init__(self):
        super().__init__('eyeball_node')
        self.get_logger().info("start eyeball")
        self.i = 0
        self.sub_eyeball = self.create_subscription(
            Float64MultiArray, 
            '/position', 
            self.listener_callback,
            10)
        self.sub_eyeball # prevent unused variable warning
        self.ctime = time.time()

    def listener_callback(self, msg):
        new_time = time.time()
        fps = 1. / (new_time - self.ctime)
        self.ctime = new_time
        self.get_logger().info('eyeball cmds:"%s" fps: "%s"' % (len(msg.data), int(fps)))


def main(args=None):
    rclpy.init(args=args)

    eyeball_subscriber = eyeballSubscriber()

    rclpy.spin(eyeball_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    eyeball_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()