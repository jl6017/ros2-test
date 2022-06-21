import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
import torch
import torch.nn as nn
import torch.nn.functional as F
PATH = "./src/nn_computer/nn_computer/I_map/model.pt"
device = torch.device('cpu')


class inverse_model(nn.Module):

    def __init__(self):
        super(inverse_model, self).__init__()

        # 59*2, 113*2
        self.inputsize = 226

        # 15
        self.outputsize = 11

        self.fc1 = nn.Linear(self.inputsize, 1024)
        self.fc2 = nn.Linear(1024, 512)
        self.fc3 = nn.Linear(512, self.outputsize)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(1024)
        self.bn3 = nn.BatchNorm1d(512)

    def forward(self, x):
        x = F.relu(self.fc1(x),inplace = True)
        x = self.bn2(x)
        x = F.relu(self.fc2(x) ,inplace = True)
        x = torch.sigmoid(self.fc3(x))

        return x

    def loss(self, pred, target):
        return torch.mean((pred - target) ** 2)


class nnSubscriber(Node):

    def __init__(self):
        super().__init__('nn_node')
        self.get_logger().info("start neural network")
        self.i = 0
        self.sub_points = self.create_subscription(
            Float64MultiArray, 
            '/normal_lmks', 
            self.listener_callback,
            10)
        self.sub_points  # prevent unused variable warning
        self.pub_eye_cmds = self.create_publisher(Float64MultiArray, '/eye_cmds', 10)
        self.pub_mouth_cmds = self.create_publisher(Float64MultiArray, '/mouth_cmds', 10)
        self.model = inverse_model()
        self.model.load_state_dict(torch.load(PATH, map_location=device))
        self.model.eval()

    def listener_callback(self, msg):
        msg_out_eye = Float64MultiArray()
        msg_out_mouth = Float64MultiArray()

        normalized_lmks_2 = np.array([msg.data[:113], msg.data[113:]])
        input_d = torch.from_numpy(np.expand_dims(normalized_lmks_2, axis=0))
        input_d = torch.flatten(input_d, 1)
        cmds = self.model.forward(input_d.float())
        msg_out_eye.data = cmds.tolist()[0][:4]
        msg_out_mouth.data = cmds.tolist()[0][4:]
        self.pub_eye_cmds.publish(msg_out_eye)
        self.pub_mouth_cmds.publish(msg_out_mouth)
        
        self.get_logger().info('nn get: "%s", out: "%s", "%s"' %(len(msg.data), len(msg_out_eye.data), len(msg_out_mouth.data)))

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