#eje coordenado real 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PositionEstimator(Node):
    def __init__(self):
        super().__init__('position_estimator')

        # Suscripción a la cámara y publicación de datos de posición
        self.subscription = self.create_subscription(
            Image, 'video_feed', self.listener_callback, 10)
        self.publisher = self.create_publisher(Point, 'velocity_input', 10)

        self.bridge = CvBridge()
        self.previous_center = None

        # Parámetros del sistema
        self.camera_height_cm = 96  # Altura de la cámara desde el suelo
        self.robot_height_cm = 8    # Altura del robot
        self.origin_offset_y = self.robot_height_cm / 100  # 0.08 m

        # Rango de color naranja en HSV
        self.lower_orange = np.array([5, 100, 100])
        self.upper_orange = np.array([15, 255, 255])

    def listener_callback(self, msg):
        try:
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Crear máscara para el color naranja
            mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Contorno más grande
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                cX, cY = x + w // 2, y + h // 2

                # Convertir píxeles a cm (ajustar escala según pruebas)
                distance_x_cm = (cX - 320) * 0.1  # 1 pixel = 0.1 cm
                distance_y_cm = (480 - cY) * 0.1

                # Calcular distancia al origen
                distance_to_origin = math.sqrt(distance_x_cm ** 2 + distance_y_cm ** 2)

                # Corrección en Y por la altura del robot
                distance_y_real = (distance_y_cm / 100) + self.origin_offset_y

                # Log de la distancia al origen
                self.get_logger().info(f'Distancia al origen: {distance_to_origin:.2f} cm')

                # Publicar las coordenadas como Point
                point_msg = Point()
                point_msg.x = float(distance_x_cm) / 100  # Convertir a metros
                point_msg.y = float(distance_y_real)      # Ya en metros
                point_msg.z = 0.0
                self.publisher.publish(point_msg)

                # Dibujar resultados en la imagen
                cv2.circle(cv_image, (cX, cY), 8, (0, 255, 0), -1)  # Punto central
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Rectángulo de detección
                cv2.putText(cv_image, f'({distance_x_cm:.2f} cm, {distance_y_real * 100:.2f} cm)', 
                            (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 0), 2)

            else:
                # Mensaje si no se detecta el robot
                cv2.putText(cv_image, 'No detection', (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Mostrar la imagen procesada
            cv2.imshow('Position Estimation', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    position_estimator = PositionEstimator()
    rclpy.spin(position_estimator)
    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
