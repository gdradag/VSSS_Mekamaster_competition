import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import sys
import tty
import termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("172.20.10.2", 1883, 60)  # Cambia "localhost" por "172.20.10.2"
        self.get_logger().info("Teleop Node started. Use WASD to control the robot.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                self.mqtt_client.publish("robot/move", "forward")
            elif key == 's':
                self.mqtt_client.publish("robot/move", "backward")
            elif key == 'a':
                self.mqtt_client.publish("robot/move", "left")
            elif key == 'd':
                self.mqtt_client.publish("robot/move", "right")
            elif key == ' ':
                self.mqtt_client.publish("robot/move", "stop")
            elif key == '\x03':  # Ctrl+C
                break

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()