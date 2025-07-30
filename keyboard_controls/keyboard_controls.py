import rclpy
from rclpy.node import Node
from tr_messages.msg import SimTeleopInput
from pynput import keyboard
import numpy

MAX_TRANSLATION_VEL = 1.0
MAX_ANGULAR_VEL = 1.0


class TeleopInput:
    def __init__(self) -> None:
        self.pitch = 0.0
        self.yaw = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angular_vel = 0.0

    def __str__(self) -> str:
        return (
            f"{self.x_vel}, {self.y_vel}, {self.angular_vel}, {self.pitch}, {self.yaw}"
        )


class KeyboardControls(Node):
    def __init__(self):
        super().__init__("keyboard_controls")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.primary_robot_pub = self.create_publisher(
            SimTeleopInput, "simulation/primary_robot_teleop", 10
        )
        self.secondary_robot_put = self.create_publisher(
            SimTeleopInput, "simulation/secondary_robot_telop", 10
        )
        self.primary_robot_teleop_input = TeleopInput()
        self.secondary_robot_teleop_input = TeleopInput()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

    def on_press(self, key: keyboard.Key):
        print("ON PRESS")
        if not hasattr(key, "char") or not key.char.isalpha():
            return

        # primary robot translation
        if key.char == "t":
            self.primary_robot_teleop_input.x_vel = MAX_TRANSLATION_VEL
        elif key.char == "g":
            self.primary_robot_teleop_input.x_vel = -MAX_TRANSLATION_VEL
        # positive y_vel is left
        elif key.char == "f":
            self.primary_robot_teleop_input.y_vel = MAX_TRANSLATION_VEL
        elif key.char == "h":
            self.primary_robot_teleop_input.y_vel = -MAX_TRANSLATION_VEL
        # primary robot angular rotation
        # positive angular velocity is counterclockwise
        elif key.char == "r":
            self.primary_robot_teleop_input.angular_vel = MAX_ANGULAR_VEL
        elif key.char == "y":
            self.primary_robot_teleop_input.angular_vel = -MAX_ANGULAR_VEL

    def on_release(self, key: keyboard.Key):
        print("ON RELEASE")
        if not hasattr(key, "char") or not key.char.isalpha():
            return

        # same as on_press but flipped signs
        # primary robot translation
        if key.char == "t":
            self.primary_robot_teleop_input.x_vel = 0.0
        elif key.char == "g":
            self.primary_robot_teleop_input.x_vel = 0.0
        elif key.char == "f":
            self.primary_robot_teleop_input.y_vel = 0.0
        elif key.char == "h":
            self.primary_robot_teleop_input.y_vel = 0.0
        # primary robot angular rotation
        elif key.char == "r":
            self.primary_robot_teleop_input.angular_vel = 0.0
        elif key.char == "y":
            self.primary_robot_teleop_input.angular_vel = 0.0

    def timer_callback(self):
        print("publishing", self.primary_robot_teleop_input)
        primary_robot_msg = SimTeleopInput()
        primary_robot_msg.x_vel = self.primary_robot_teleop_input.x_vel
        primary_robot_msg.y_vel = self.primary_robot_teleop_input.y_vel
        primary_robot_msg.angular_vel = self.primary_robot_teleop_input.angular_vel

        self.primary_robot_pub.publish(primary_robot_msg)


def main(args=None):
    rclpy.init()
    node = KeyboardControls()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
