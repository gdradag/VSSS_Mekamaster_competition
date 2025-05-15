#nodo calculo de velocidades 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import math

class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')

        self.subscription = self.create_subscription(
            Point, 'velocity_input', self.listener_callback, 10)

        self.linear_pub = self.create_publisher(Float32, 'linear_velocity', 10)
        self.angular_pub = self.create_publisher(Float32, 'angular_velocity', 10)

        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None

        self.angle_threshold = 0.05  # Umbral para ruido angular en radianes

    def listener_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()
        current_time_sec = current_time[0] + current_time[1] * 1e-9

        if self.prev_time is not None:
            # Calcular diferencias de tiempo y posición
            dt = current_time_sec - self.prev_time
            dx = msg.x - self.prev_x
            dy = msg.y - self.prev_y

            if dt > 0.0:
                # Velocidad Lineal
                linear_velocity = math.sqrt(dx ** 2 + dy ** 2) / dt

                # Ángulo Actual
                current_theta = math.atan2(dy, dx)

                # Cambio en el ángulo
                dtheta = current_theta - self.prev_theta

                # Normalizar el ángulo entre -pi y pi
                while dtheta > math.pi:
                    dtheta -= 2 * math.pi
                while dtheta < -math.pi:
                    dtheta += 2 * math.pi

                # Velocidad Angular
                angular_velocity = dtheta / dt

                # Aplicar umbral dinámico basado en dt
                if abs(angular_velocity) < (self.angle_threshold / dt):
                    angular_velocity = 0.0

                # Publicar velocidades
                self.publish_velocity(linear_velocity, angular_velocity)

        # Actualizar valores previos
        self.prev_time = current_time_sec
        self.prev_x = msg.x
        self.prev_y = msg.y
        self.prev_theta = math.atan2(msg.y, msg.x)

    def publish_velocity(self, linear_velocity, angular_velocity):
        # Publicar velocidad lineal
        linear_msg = Float32()
        linear_msg.data = linear_velocity
        self.linear_pub.publish(linear_msg)

        # Publicar velocidad angular
        angular_msg = Float32()
        angular_msg.data = angular_velocity
        self.angular_pub.publish(angular_msg)

        # Mostrar en consola
        self.get_logger().info(f'Linear Velocity: {linear_velocity:.2f} m/s')
        self.get_logger().info(f'Angular Velocity: {angular_velocity:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    velocity_calculator = VelocityCalculator()
    rclpy.spin(velocity_calculator)
    velocity_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()