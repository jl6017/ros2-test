import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
import mediapipe as mp

from .normalization import human2robot, reorder_lmks, matric2simple, R_static_face, robot_edge
from .lmks_data.face_geometry import (
    PCF,
    get_metric_landmarks,
    procrustes_landmark_basis,
)

class mpNode(Node):
    """
    mediapipe input node
    """
    def __init__(self):
        super().__init__('mp_node')
        self.pub_points = self.create_publisher(Float64MultiArray, '/simple_lmks', 10)
        self.i = 0
        self.run_pub()


    def run_pub(self):
        msg = Float64MultiArray()

        mp_face_mesh = mp.solutions.face_mesh
        frame_width, frame_height = 1920, 1080
        channels = 3
        focal_length = frame_width
        center = (frame_width / 2, frame_height / 2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]],
            dtype="double",
        )
        pcf = PCF(
            near=1,
            far=10000,
            frame_height=frame_height,
            frame_width=frame_width,
            fy=camera_matrix[1, 1],
        )

        face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5)
        cap = cv2.VideoCapture(0)

        while True:
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                return 0, 0 
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(image)
            image = cv2.resize(image,(960,540))
            # rgb_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # cv2.namedWindow("RGB window")
            # cv2.imshow("Input", rgb_image)
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    landmarks = np.array(
                                [(lm.x, lm.y, lm.z) for lm in face_landmarks.landmark]
                            )
                    landmarks = landmarks.T
                    landmarks = landmarks[:, :468]

                    metric_landmarks, _ = get_metric_landmarks(
                        landmarks.copy(), pcf
                    )

                    simple_lmks = matric2simple(metric_landmarks)
                    
                    msg.data = np.reshape(simple_lmks, 226).tolist()
                    self.pub_points.publish(msg)
                    self.get_logger().info('Publishing: "%s"' % len(msg.data))
                    self.i += 1


def main(args=None):

    rclpy.init(args=args)
    mp_node = mpNode()
    rclpy.spin(mp_node)
    rclpy.shutdown()
    mp_node.destroy_node()


if __name__ == '__main__':
    main()


    # ros2 pkg create mp_input --build-type ament_python --dependencies rclpy
    # ros2 pkg create nn_computer --build-type ament_python --dependencies rclpy
    # ros2 pkg create cmd_output --build-type ament_python --dependencies rclpy

    # ros2 run mp_test2 mpinput_node