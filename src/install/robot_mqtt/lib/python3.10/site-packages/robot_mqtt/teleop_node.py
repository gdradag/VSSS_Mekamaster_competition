import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from std_msgs.msg import String

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Configurar el cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.8.100", 1883, 60)  # Cambia "localhost" por la IP de tu broker MQTT
        self.get_logger().info("Teleop Node started. Listening to teleop_cmd topic.")

        # Suscriptor al tópico 'teleop_cmd'
        self.create_subscription(
            String,
            'teleop_cmd',
            self.teleop_cmd_callback,
            10
        )

    def teleop_cmd_callback(self, msg):
        """Callback para recibir los comandos del nodo de interfaz y enviarlos por MQTT."""
        command = msg.data
        self.get_logger().info(f"Comando recibido: {command}")

        # Publicar el comando recibido en el broker MQTT
        self.mqtt_client.publish("robot/move", command)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    # Ejecutar ROS 2
    rclpy.spin(teleop_node)

    # Limpiar después de que termine
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

