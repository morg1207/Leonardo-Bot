import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Publicadores para imagen sin comprimir y comprimida
        self.raw_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.compressed_publisher = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)

        self.device = 0
        self.fps = 30
        self.init_param()

        # Captura de video
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            self.get_logger().warn(f"No se puede abrir el dispositivo de video {self.device}")
        
        self.br = CvBridge()
        rate = 1/self.fps
        self.timer = self.create_timer(rate, self.timer_callback)  # Publicación periódica

    def init_param(self):
        self.declare_parameter('device', 0)
        self.device = self.get_parameter('device').get_parameter_value().integer_value
        self.get_logger().info(f"Video device: {self.device}")

        self.declare_parameter('fps', 30)
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.get_logger().info(f"Frames por segundo: {self.fps}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Publicar imagen sin comprimir
            img_msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            self.raw_publisher.publish(img_msg)

            # Comprimir la imagen antes de publicarla
            compressed_msg = CompressedImage()
            compressed_msg.header = img_msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()

            # Publicar imagen comprimida
            self.compressed_publisher.publish(compressed_msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
