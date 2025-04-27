import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.publisher = self.create_publisher(String, "ros2_commands", 10)
        self.get_logger().info("Nodo de movimiento iniciado.")

    def move_forward(self):
        msg = String()
        msg.data = "forward"
        self.publisher.publish(msg)
        self.get_logger().info("Enviando comando: forward")

    def move_backward(self):
        msg = String()
        msg.data = "backward"
        self.publisher.publish(msg)
        self.get_logger().info("Enviando comando: backward")

    def stop(self):
        msg = String()
        msg.data = "stop"
        self.publisher.publish(msg)
        self.get_logger().info("Enviando comando: stop")

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()

    # Ejemplo de secuencia de movimiento
    node.move_forward()
    time.sleep(2)
    node.move_backward()
    time.sleep(2)
    node.stop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()