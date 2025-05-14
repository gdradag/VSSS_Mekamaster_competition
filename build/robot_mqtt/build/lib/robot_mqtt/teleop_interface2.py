import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import Label, Button
import cv2
from PIL import Image, ImageTk

class TeleopInterface(Node):
    def __init__(self):
        super().__init__('teleop_interface')
        self.publisher_1 = self.create_publisher(String, 'teleop_cmd3', 10)  # Robot 1
        self.publisher_2 = self.create_publisher(String, 'teleop_cmd4', 10)  # Robot 2
        self.init_ui()

    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("Teleoperación de Robots VSSS")

        # Etiqueta de título
        self.label = Label(self.root, text="Control de Robots VSSS", font=("Arial", 16))
        self.label.pack()

        # Video de la cámara
        self.video_label = Label(self.root)
        self.video_label.pack()

        # Marco para los botones
        button_frame = tk.Frame(self.root)
        button_frame.pack()

        # Títulos de los robots
        self.robot1_label = Label(button_frame, text="Robot 1", font=("Arial", 14))
        self.robot1_label.grid(row=0, column=1, pady=5)
        self.robot2_label = Label(button_frame, text="Robot 2", font=("Arial", 14))
        self.robot2_label.grid(row=0, column=4, pady=5)

        # Botones de control para el Robot 1
        self.btn_forward_1 = Button(button_frame, text="↑", bg="green", fg="white", font=("Arial", 12, "bold"),
                                   command=lambda: self.send_command("forward", 1))
        self.btn_left_1 = Button(button_frame, text="←", bg="green", fg="white", font=("Arial", 12, "bold"),
                                command=lambda: self.send_command("left", 1))
        self.btn_stop_1 = Button(button_frame, text="STOP", bg="red", fg="white", font=("Arial", 12, "bold"),
                                command=lambda: self.send_command("stop", 1))
        self.btn_right_1 = Button(button_frame, text="→", bg="green", fg="white", font=("Arial", 12, "bold"),
                                 command=lambda: self.send_command("right", 1))
        self.btn_backward_1 = Button(button_frame, text="↓", bg="green", fg="white", font=("Arial", 12, "bold"),
                                    command=lambda: self.send_command("backward", 1))

        # Botones de control para el Robot 2
        self.btn_forward_2 = Button(button_frame, text="↑", bg="blue", fg="white", font=("Arial", 12, "bold"),
                                   command=lambda: self.send_command("forward", 2))
        self.btn_left_2 = Button(button_frame, text="←", bg="blue", fg="white", font=("Arial", 12, "bold"),
                                command=lambda: self.send_command("left", 2))
        self.btn_stop_2 = Button(button_frame, text="STOP", bg="red", fg="white", font=("Arial", 12, "bold"),
                                command=lambda: self.send_command("stop", 2))
        self.btn_right_2 = Button(button_frame, text="→", bg="blue", fg="white", font=("Arial", 12, "bold"),
                                 command=lambda: self.send_command("right", 2))
        self.btn_backward_2 = Button(button_frame, text="↓", bg="blue", fg="white", font=("Arial", 12, "bold"),
                                    command=lambda: self.send_command("backward", 2))

        # Distribución con grid() para el control de Robot 1
        self.btn_forward_1.grid(row=1, column=1, padx=5, pady=5)
        self.btn_left_1.grid(row=2, column=0, padx=5, pady=5)
        self.btn_stop_1.grid(row=2, column=1, padx=5, pady=5)
        self.btn_right_1.grid(row=2, column=2, padx=5, pady=5)
        self.btn_backward_1.grid(row=3, column=1, padx=5, pady=5)

        # Distribución con grid() para el control de Robot 2
        self.btn_forward_2.grid(row=1, column=4, padx=5, pady=5)
        self.btn_left_2.grid(row=2, column=3, padx=5, pady=5)
        self.btn_stop_2.grid(row=2, column=4, padx=5, pady=5)
        self.btn_right_2.grid(row=2, column=5, padx=5, pady=5)
        self.btn_backward_2.grid(row=3, column=4, padx=5, pady=5)

        # Captura de la cámara
        self.cap = cv2.VideoCapture(0)
        self.update_frame()

        # Asignar teclas de control para el Robot 1
        self.root.bind("<w>", lambda event: self.send_command("forward", 1))
        self.root.bind("<a>", lambda event: self.send_command("left", 1))
        self.root.bind("<s>", lambda event: self.send_command("backward", 1))
        self.root.bind("<d>", lambda event: self.send_command("right", 1))
        self.root.bind("<space>", lambda event: self.send_command("stop", 1))

        # Asignar teclas de control para el Robot 2
        self.root.bind("<Up>", lambda event: self.send_command("forward", 2))
        self.root.bind("<Left>", lambda event: self.send_command("left", 2))
        self.root.bind("<Down>", lambda event: self.send_command("backward", 2))
        self.root.bind("<Right>", lambda event: self.send_command("right", 2))
        self.root.bind("<Return>", lambda event: self.send_command("stop", 2))

        # Integración con ROS 2
        self.root.after(100, self.ros_spin)

    def send_command(self, command, robot):
        msg = String()
        if robot == 1:
            msg.data = command
            self.publisher_1.publish(msg)
            self.get_logger().info(f"Enviado comando al Robot 1: {command}")
        elif robot == 2:
            msg.data = command
            self.publisher_2.publish(msg)
            self.get_logger().info(f"Enviado comando al Robot 2: {command}")

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        self.root.after(10, self.update_frame)

    def ros_spin(self):
        """Permite que ROS 2 procese mensajes sin bloquear Tkinter."""
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.ros_spin)

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.root.quit()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopInterface()
    node.root.mainloop()  # Ejecuta Tkinter y ROS 2 sin bloquearse
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
