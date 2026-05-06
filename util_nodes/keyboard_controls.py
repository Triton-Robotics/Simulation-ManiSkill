#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tr_messages.msg import SimTeleopInput, ChassisControl
from pynput import keyboard
import numpy
from sim_node import constants


class TeleopInput:
    def __init__(self) -> None:
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0
        self.roll_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0
        self.yaw_vel = 0.0
        self.turret_pitch = 0.0

    def __str__(self) -> str:
        return (
            f"{self.x_vel}, {self.y_vel}, {self.z_vel},"
            f"{self.roll_vel}, {self.pitch_vel}, {self.yaw_vel},"
            f"{self.yaw_vel}, {self.turret_pitch}"
        )


class KeyboardControls(Node):
    def __init__(self):
        super().__init__("keyboard_controls")

        # TODO probably make this slower
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.primary_robot_pub = self.create_publisher(
            ChassisControl, "simulation/primary_robot_teleop", 10
        )
        self.secondary_robot_pub = self.create_publisher(
            ChassisControl, "simulation/secondary_robot_teleop", 10
        )
        self.primary_robot_teleop_input = TeleopInput()
        self.secondary_robot_teleop_input = TeleopInput()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

    ALLOWED_CHARS = set("tgfhryijklounm,.1234567890")
    def on_press(self, key: keyboard.Key):
        if not hasattr(key, "char") or key.char is None:
            return None
        if key.char not in self.ALLOWED_CHARS:
            return

        # primary robot translation
        if key.char == "t":
            self.primary_robot_teleop_input.x_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif key.char == "g":
            self.primary_robot_teleop_input.x_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        # positive y_vel is left
        elif key.char == "f":
            self.primary_robot_teleop_input.y_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif key.char == "h":
            self.primary_robot_teleop_input.y_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        # primary robot angular rotation
        # positive angular velocity is counterclockwise
        elif key.char == "r":
            self.primary_robot_teleop_input.yaw_vel = (
                constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        elif key.char == "y":
            self.primary_robot_teleop_input.yaw_vel = (
                -constants.TELEOP_ANGULAR_VEL_RADS_S
            )

        # secondary robot translation
        if key.char == "i":
            self.secondary_robot_teleop_input.x_vel = (
                constants.TELEOP_TRANSLATION_VEL_M_S
            )
        elif key.char == "k":
            self.secondary_robot_teleop_input.x_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        # positive y_vel is left
        elif key.char == "j":
            self.secondary_robot_teleop_input.y_vel = (
                constants.TELEOP_TRANSLATION_VEL_M_S
            )
        elif key.char == "l":
            self.secondary_robot_teleop_input.y_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        elif key.char == "u":
            self.secondary_robot_teleop_input.z_vel = (
                constants.TELEOP_TRANSLATION_VEL_M_S
            )
        elif key.char == "o":
            self.secondary_robot_teleop_input.z_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        elif key.char == "n":
            self.secondary_robot_teleop_input.roll_vel = (
                constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        elif key.char == "m":
            self.secondary_robot_teleop_input.roll_vel = (
                -constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        elif key.char == ",":
            self.secondary_robot_teleop_input.pitch_vel = (
                constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        elif key.char == ".":
            self.secondary_robot_teleop_input.pitch_vel = (
                -constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        # secondary robot angular rotation
        # this is coded this way because secondary robot is used for target practice so we often want to beyblade it at different speeds
        elif key.char == "5":
            self.secondary_robot_teleop_input.yaw_vel = 0.0
        elif key.char == "4":
            self.secondary_robot_teleop_input.yaw_vel = (
                constants.MAX_ANGULAR_VEL_RADS_S * 0.25
            )
        elif key.char == "3":
            self.secondary_robot_teleop_input.yaw_vel = (
                constants.MAX_ANGULAR_VEL_RADS_S * 0.5
            )
        elif key.char == "2":
            self.secondary_robot_teleop_input.yaw_vel = (
                constants.MAX_ANGULAR_VEL_RADS_S * 0.75
            )
        elif key.char == "1":
            self.secondary_robot_teleop_input.yaw_vel = (
                constants.MAX_ANGULAR_VEL_RADS_S
            )
        elif key.char == "6":
            self.secondary_robot_teleop_input.yaw_vel = (
                -constants.MAX_ANGULAR_VEL_RADS_S * 0.25
            )
        elif key.char == "7":
            self.secondary_robot_teleop_input.yaw_vel = (
                -constants.MAX_ANGULAR_VEL_RADS_S * 0.5
            )
        elif key.char == "8":
            self.secondary_robot_teleop_input.yaw_vel = (
                -constants.MAX_ANGULAR_VEL_RADS_S * 0.75
            )
        elif key.char == "9":
            self.secondary_robot_teleop_input.yaw_vel = (
                -constants.MAX_ANGULAR_VEL_RADS_S
            )

    def on_release(self, key: keyboard.Key):
        if not hasattr(key, "char") or key.char is None:
            return None
        if key.char not in self.ALLOWED_CHARS:
            return

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
            self.primary_robot_teleop_input.yaw_vel = 0.0
        elif key.char == "y":
            self.primary_robot_teleop_input.yaw_vel = 0.0

        # secondary robot translation
        if key.char == "i":
            self.secondary_robot_teleop_input.x_vel = 0.0
        elif key.char == "k":
            self.secondary_robot_teleop_input.x_vel = 0.0
        elif key.char == "j":
            self.secondary_robot_teleop_input.y_vel = 0.0
        elif key.char == "l":
            self.secondary_robot_teleop_input.y_vel = 0.0
        elif key.char == "u":
            self.secondary_robot_teleop_input.z_vel = 0.0
        elif key.char == "o":
            self.secondary_robot_teleop_input.z_vel = 0.0
        elif key.char == "n":
            self.secondary_robot_teleop_input.roll_vel = 0.0
        elif key.char == "m":
            self.secondary_robot_teleop_input.roll_vel = 0.0
        elif key.char == ",":
            self.secondary_robot_teleop_input.pitch_vel = 0.0
        elif key.char == ".":
            self.secondary_robot_teleop_input.pitch_vel = 0.0
        

    def timer_callback(self):
        # print(
        #     "publishing",
        #     "primary: "
        #     self.primary_robot_teleop_input,
        #     "secondary: ",
        #     self.secondary_robot_teleop_input,
        # )
        primary_robot_msg = ChassisControl()
        primary_robot_msg.x_vel = self.primary_robot_teleop_input.x_vel
        primary_robot_msg.y_vel = self.primary_robot_teleop_input.y_vel
        primary_robot_msg.yaw_vel = self.primary_robot_teleop_input.yaw_vel
        self.primary_robot_pub.publish(primary_robot_msg)

        secondary_robot_msg = ChassisControl()
        secondary_robot_msg.x_vel = self.secondary_robot_teleop_input.x_vel
        secondary_robot_msg.y_vel = self.secondary_robot_teleop_input.y_vel
        secondary_robot_msg.z_vel = self.secondary_robot_teleop_input.z_vel
        secondary_robot_msg.roll_vel = self.secondary_robot_teleop_input.roll_vel
        secondary_robot_msg.pitch_vel = self.secondary_robot_teleop_input.pitch_vel
        secondary_robot_msg.yaw_vel = self.secondary_robot_teleop_input.yaw_vel
        secondary_robot_msg.turret_pitch = 0.0
        secondary_robot_msg.turret_yaw = 0.0
        #secondary_robot_msg.yaw_vel = self.secondary_robot_teleop_input.yaw_vel
        self.secondary_robot_pub.publish(secondary_robot_msg)


def main(args=None):
    print("running sim node keyboard controls")
    rclpy.init()
    node = KeyboardControls()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    print("shutting down sim node keyboard controls")
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()   
