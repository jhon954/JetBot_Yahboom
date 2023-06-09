import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.image_callback,
            10)
        self.subscription

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Aplica tu algoritmo de visión artificial utilizando OpenCV a cv_image

        # Muestra la imagen procesada (opcional)
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
