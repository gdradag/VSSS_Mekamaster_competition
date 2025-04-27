# nodo_video_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_feed', 10)
        self.bridge = CvBridge()
        
        # Configurar cámara
        self.cap = cv2.VideoCapture(2)  # Usar índice correcto para tu cámara
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara')
            exit()
            
        # Configurar resolución
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = self.create_timer(0.033, self.publish_frame)  # ~30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convertir y publicar frame
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error al publicar video: {str(e)}')
        else:
            self.get_logger().warn('No se pudo leer frame de la cámara')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    publisher = VideoPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()