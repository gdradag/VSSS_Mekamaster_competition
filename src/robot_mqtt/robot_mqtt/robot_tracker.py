import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import paho.mqtt.client as mqtt

class RobotTracker(Node):
    def __init__(self):
        super().__init__('robot_tracker')

        # Suscripción a la cámara y publicación de datos de control
        self.subscription = self.create_subscription(
            Image, 'video_feed', self.image_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mqtt_publisher = self.create_publisher(String, 'mqtt_command', 10)
        
        # Configuración MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("172.20.10.2", 1883, 60)

        self.bridge = CvBridge()
        
        # Parámetros del sistema
        self.camera_height_cm = 96
        self.field_width_cm = 150  # Ajustar según el tamaño real del campo
        self.field_height_cm = 130  # Ajustar según el tamaño real del campo
        
        # Parámetros de control
        self.kp_linear = 0.5     # Ganancia proporcional para velocidad lineal
        self.kp_angular = 1.0    # Ganancia proporcional para velocidad angular
        self.ki_linear = 0.1     # Ganancia integral para velocidad lineal
        self.ki_angular = 0.2    # Ganancia integral para velocidad angular
        self.kd_linear = 0.05    # Ganancia derivativa para velocidad lineal
        self.kd_angular = 0.1    # Ganancia derivativa para velocidad angular
        
        # Variables de control PID
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        
        # Rangos de color en HSV
        # Rango HSV para el robot (morado)
        self.lower_purple = np.array([130, 50, 50])
        self.upper_purple = np.array([170, 255, 255])
        # Rango HSV para el objetivo (naranja)
        self.lower_orange = np.array([5, 100, 100])
        self.upper_orange = np.array([15, 255, 255])
        
        # Dimensiones de la imagen
        self.img_width = 640
        self.img_height = 480
        
        # Variables de estado
        self.robot_position = None
        self.target_position = None
        self.robot_orientation = 0.0
        self.prev_robot_position = None
        self.prev_time = None
        
        self.get_logger().info('Robot Tracker Node initialized.')

    def image_callback(self, msg):
        try:
            # Obtener timestamp actual
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time_sec = current_time[0] + current_time[1] * 1e-9
            
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.img_width, self.img_height))
            
            # Crear una imagen para visualización
            display_img = cv_image.copy()
            
            # Convertir a HSV para detección de colores
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detectar robot (morado)
            robot_mask = cv2.inRange(hsv, self.lower_purple, self.upper_purple)
            robot_mask = self.process_mask(robot_mask)
            robot_contours, _ = cv2.findContours(robot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Detectar objetivo (naranja)
            target_mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
            target_mask = self.process_mask(target_mask)
            target_contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Dibujar sistema de coordenadas
            self.draw_coordinate_system(display_img)
            
            # Si se detecta tanto el robot como el objetivo
            if robot_contours and target_contours:
                # Encontrar el contorno más grande para el robot
                robot_contour = max(robot_contours, key=cv2.contourArea)
                # Encontrar el contorno más grande para el objetivo
                target_contour = max(target_contours, key=cv2.contourArea)
                
                # Calcular posición del robot
                robot_x, robot_y, robot_width, robot_height = cv2.boundingRect(robot_contour)
                robot_cx = robot_x + robot_width // 2
                robot_cy = robot_y + robot_height // 2
                
                # Calcular orientación del robot (encontrar el punto más lejano)
                robot_moments = cv2.moments(robot_contour)
                if robot_moments["m00"] != 0:
                    robot_cx = int(robot_moments["m10"] / robot_moments["m00"])
                    robot_cy = int(robot_moments["m01"] / robot_moments["m00"])
                
                # Calcular orientación usando momentos
                if self.prev_robot_position is not None and current_time_sec != self.prev_time:
                    prev_x, prev_y = self.prev_robot_position
                    dx = robot_cx - prev_x
                    dy = robot_cy - prev_y
                    
                    # Solo actualizar orientación si el robot se movió significativamente
                    if dx*dx + dy*dy > 5:
                        self.robot_orientation = math.atan2(dy, dx)
                
                # Convertir coordenadas de píxeles a centímetros
                robot_x_cm = self.pixel_to_cm_x(robot_cx)
                robot_y_cm = self.pixel_to_cm_y(robot_cy)
                
                # Calcular posición del objetivo
                target_x, target_y, target_width, target_height = cv2.boundingRect(target_contour)
                target_cx = target_x + target_width // 2
                target_cy = target_y + target_height // 2
                
                # Convertir coordenadas de píxeles a centímetros
                target_x_cm = self.pixel_to_cm_x(target_cx)
                target_y_cm = self.pixel_to_cm_y(target_cy)
                
                # Actualizar posiciones
                self.robot_position = (robot_cx, robot_cy)
                self.target_position = (target_cx, target_cy)
                
                # Calcular el vector desde el robot al objetivo
                direction_x = target_x_cm - robot_x_cm
                direction_y = target_y_cm - robot_y_cm
                
                # Calcular distancia euclidiana al objetivo
                distance_to_target = math.sqrt(direction_x**2 + direction_y**2)
                
                # Calcular ángulo deseado hacia el objetivo
                desired_angle = math.atan2(direction_y, direction_x)
                
                # Calcular error angular (diferencia entre orientación actual y deseada)
                angle_error = self.normalize_angle(desired_angle - self.robot_orientation)
                
                # Aplicar control PID
                control_signals = self.calculate_pid_control(distance_to_target, angle_error, current_time_sec)
                linear_velocity, angular_velocity = control_signals
                
                # Publicar comandos de velocidad
                self.publish_velocity_commands(linear_velocity, angular_velocity)
                
                # Calcular comando MQTT para los motores
                mqtt_command = self.calculate_motor_commands(linear_velocity, angular_velocity)
                self.mqtt_client.publish("robot/move", mqtt_command)
                
                # Visualizar detecciones y trayectoria
                self.visualize_tracking(display_img, robot_contour, target_contour, 
                                      (robot_cx, robot_cy), (target_cx, target_cy),
                                      distance_to_target, angle_error)
                
                # Actualizar variables para el siguiente ciclo
                self.prev_robot_position = (robot_cx, robot_cy)
                self.prev_time = current_time_sec
                
            elif robot_contours:
                # Solo se detecta el robot
                robot_contour = max(robot_contours, key=cv2.contourArea)
                robot_x, robot_y, robot_width, robot_height = cv2.boundingRect(robot_contour)
                robot_cx = robot_x + robot_width // 2
                robot_cy = robot_y + robot_height // 2
                
                # Usar color morado (128, 0, 128) para visualización en BGR (128, 0, 128)
                cv2.circle(display_img, (robot_cx, robot_cy), 8, (128, 0, 128), -1)
                cv2.rectangle(display_img, (robot_x, robot_y), (robot_x + robot_width, robot_y + robot_height), (128, 0, 128), 2)
                cv2.putText(display_img, "Robot", (robot_cx + 10, robot_cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 128), 2)
                
                # Detener el robot si no se ve el objetivo
                self.mqtt_client.publish("robot/move", "stop")
                
            elif target_contours:
                # Solo se detecta el objetivo
                target_contour = max(target_contours, key=cv2.contourArea)
                target_x, target_y, target_width, target_height = cv2.boundingRect(target_contour)
                target_cx = target_x + target_width // 2
                target_cy = target_y + target_height // 2
                
                cv2.circle(display_img, (target_cx, target_cy), 8, (0, 165, 255), -1)
                cv2.rectangle(display_img, (target_x, target_y), (target_x + target_width, target_y + target_height), (0, 165, 255), 2)
                cv2.putText(display_img, "Objetivo", (target_cx + 10, target_cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                
                # Detener el robot si no se ve a sí mismo
                self.mqtt_client.publish("robot/move", "stop")
            
            else:
                # No se detecta ni robot ni objetivo
                cv2.putText(display_img, "No detection", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # Detener el robot
                self.mqtt_client.publish("robot/move", "stop")
            
            # Mostrar la imagen procesada
            cv2.imshow('Robot Tracking', display_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')
    
    def process_mask(self, mask):
        """Aplica operaciones morfológicas para mejorar la detección"""
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
    
    def pixel_to_cm_x(self, pixel_x):
        """Convierte coordenadas X de píxeles a centímetros"""
        # Ajustar según calibración real
        center_x = self.img_width / 2
        return (pixel_x - center_x) * (self.field_width_cm / self.img_width)
    
    def pixel_to_cm_y(self, pixel_y):
        """Convierte coordenadas Y de píxeles a centímetros"""
        # Ajustar según calibración real
        return (self.img_height - pixel_y) * (self.field_height_cm / self.img_height)
    
    def draw_coordinate_system(self, image):
        """Dibuja el sistema de coordenadas en la imagen"""
        center_x = self.img_width // 2
        center_y = self.img_height // 2
        
        # Eje X (rojo)
        cv2.line(image, (center_x, center_y), (center_x + 100, center_y), (0, 0, 255), 2)
        cv2.putText(image, "X", (center_x + 110, center_y + 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Eje Y (verde)
        cv2.line(image, (center_x, center_y), (center_x, center_y - 100), (0, 255, 0), 2)
        cv2.putText(image, "Y", (center_x - 10, center_y - 110), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Origen
        cv2.circle(image, (center_x, center_y), 5, (255, 255, 255), -1)
        cv2.putText(image, "O", (center_x - 20, center_y + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    def normalize_angle(self, angle):
        """Normaliza un ángulo al rango [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_pid_control(self, distance_error, angle_error, current_time):
        """Calcula las señales de control usando PID"""
        # Control proporcional
        p_term_linear = self.kp_linear * distance_error
        p_term_angular = self.kp_angular * angle_error
        
        # Control integral
        self.integral_linear += distance_error
        self.integral_angular += angle_error
        i_term_linear = self.ki_linear * self.integral_linear
        i_term_angular = self.ki_angular * self.integral_angular
        
        # Control derivativo
        d_term_linear = self.kd_linear * (distance_error - self.prev_error_linear)
        d_term_angular = self.kd_angular * (angle_error - self.prev_error_angular)
        
        # Calcular velocidades
        linear_velocity = p_term_linear + i_term_linear + d_term_linear
        angular_velocity = p_term_angular + i_term_angular + d_term_angular
        
        # Limitar velocidades
        linear_velocity = min(max(linear_velocity, 0.0), 0.5)  # Límites ajustables
        angular_velocity = min(max(angular_velocity, -1.0), 1.0)  # Límites ajustables
        
        # Actualizar errores previos
        self.prev_error_linear = distance_error
        self.prev_error_angular = angle_error
        
        return linear_velocity, angular_velocity
    
    def publish_velocity_commands(self, linear_velocity, angular_velocity):
        """Publica comandos de velocidad en ROS"""
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_velocity)
        twist_msg.angular.z = float(angular_velocity)
        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info(f'Cmd Vel: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}')
    
    def calculate_motor_commands(self, linear_velocity, angular_velocity):
        """Calcula comandos para motores basado en velocidades lineal y angular"""
        # Si no hay movimiento, detener
        if abs(linear_velocity) < 0.05 and abs(angular_velocity) < 0.05:
            return "stop"
        
        # Para giros cerrados
        if abs(angular_velocity) > 0.5 and abs(linear_velocity) < 0.1:
            if angular_velocity > 0:
                return "left"
            else:
                return "right"
        
        # Para movimiento adelante/atrás con giro
        if linear_velocity > 0:
            if angular_velocity > 0.2:
                return "forward_left"  # Necesitarías implementar estos comandos en el ESP32
            elif angular_velocity < -0.2:
                return "forward_right"  # Necesitarías implementar estos comandos en el ESP32
            else:
                return "forward"
        else:
            if angular_velocity > 0.2:
                return "backward_left"  # Necesitarías implementar estos comandos en el ESP32
            elif angular_velocity < -0.2:
                return "backward_right"  # Necesitarías implementar estos comandos en el ESP32
            else:
                return "backward"
    
    def visualize_tracking(self, image, robot_contour, target_contour, robot_pos, target_pos, distance, angle_error):
        """Visualiza el seguimiento y la información de control"""
        robot_cx, robot_cy = robot_pos
        target_cx, target_cy = target_pos
        
        # Dibujar contornos
        cv2.drawContours(image, [robot_contour], -1, (128, 0, 128), 2)  # Contorno morado para el robot
        cv2.drawContours(image, [target_contour], -1, (0, 165, 255), 2)  # Contorno naranja para el objetivo
        
        # Dibujar centros
        cv2.circle(image, (robot_cx, robot_cy), 8, (128, 0, 128), -1)  # Centro morado para el robot
        cv2.circle(image, (target_cx, target_cy), 8, (0, 165, 255), -1)  # Centro naranja para el objetivo
        
        # Dibujar línea de trayectoria
        cv2.line(image, (robot_cx, robot_cy), (target_cx, target_cy), (0, 255, 0), 2)
        
        # Dibujar vector de orientación del robot
        orientation_x = int(robot_cx + 30 * math.cos(self.robot_orientation))
        orientation_y = int(robot_cy + 30 * math.sin(self.robot_orientation))
        cv2.line(image, (robot_cx, robot_cy), (orientation_x, orientation_y), (255, 0, 255), 2)
        
        # Mostrar información
        cv2.putText(image, f"Robot", (robot_cx + 10, robot_cy), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 128), 2)
        cv2.putText(image, f"Target", (target_cx + 10, target_cy), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        
        # Mostrar métricas
        cv2.putText(image, f"Dist: {distance:.2f} cm", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f"Angle err: {math.degrees(angle_error):.2f} deg", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f"Robot orient: {math.degrees(self.robot_orientation):.2f} deg", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    robot_tracker = RobotTracker()
    rclpy.spin(robot_tracker)
    robot_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()