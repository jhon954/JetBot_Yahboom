import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().info(str(e))
            return

        # Preprocesamiento de la imagen (ajuste de brillo/contraste, filtrado, etc.)
        # ...

        # Detección de elementos circulares
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray_image,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=5,
            maxRadius=100
        )

        if circles is not None:
            # Dibujar los círculos encontrados en la imagen
            for circle in circles[0]:
                center = (int(circle[0]), int(circle[1]))
                radius = int(circle[2])
                cv2.circle(cv_image, center, radius, (0, 255, 0), 2)

        # Mostrar la imagen con los círculos detectados (opcional)
        cv2.imshow('Vision', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
