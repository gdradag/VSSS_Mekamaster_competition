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
        self.publisher_ = self.create_publisher(String, 'teleop_cmd', 10)
        self.init_ui()

    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("Teleoperación del Robot VSSS")

        # Etiqueta de título
        self.label = Label(self.root, text="Control del Robot", font=("Arial", 14))
        self.label.pack()

        # Video de la cámara
        self.video_label = Label(self.root)
        self.video_label.pack()

        # Marco para los botones
        button_frame = tk.Frame(self.root)
        button_frame.pack()

        # Botones de control
        self.btn_forward = Button(button_frame, text="↑", bg="green", fg="white", font=("Arial", 12, "bold"),
                                  command=lambda: self.send_command("forward"))
        self.btn_left = Button(button_frame, text="←", bg="green", fg="white", font=("Arial", 12, "bold"),
                               command=lambda: self.send_command("left"))
        self.btn_stop = Button(button_frame, text="STOP", bg="red", fg="white", font=("Arial", 12, "bold"),
                               command=lambda: self.send_command("stop"))
        self.btn_right = Button(button_frame, text="→", bg="green", fg="white", font=("Arial", 12, "bold"),
                                command=lambda: self.send_command("right"))
        self.btn_backward = Button(button_frame, text="↓", bg="green", fg="white", font=("Arial", 12, "bold"),
                                   command=lambda: self.send_command("backward"))

        # Distribución con grid()
        self.btn_forward.grid(row=0, column=1, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_backward.grid(row=2, column=1, padx=5, pady=5)

        # Captura de la cámara
        self.cap = cv2.VideoCapture(0)
        self.update_frame()

        # Asignar teclas de control
        self.root.bind("<w>", lambda event: self.send_command("forward"))
        self.root.bind("<a>", lambda event: self.send_command("left"))
        self.root.bind("<s>", lambda event: self.send_command("backward"))
        self.root.bind("<d>", lambda event: self.send_command("right"))
        self.root.bind("<space>", lambda event: self.send_command("stop"))

        # Integración con ROS 2
        self.root.after(100, self.ros_spin)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Enviado comando: {command}")

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
