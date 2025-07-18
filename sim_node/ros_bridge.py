import time
import rclpy
from rclpy.node import Node
from tr_messages.srv import WriteSerial, ListenSerial
from sensor_msgs.msg import Image
from sim_node import simulation


class Sim_Node(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.write_service = self.create_service(
            WriteSerial, "write_robot_state", self.write_robot_state
        )
        self.listen_service = self.create_service(
            ListenSerial, "read_robot_state", self.read_robot_state
        )
        self.image_pub = self.create_publisher(Image, "camera/image", 10)
        # 10ms
        self.simulation_timer = self.create_timer(0.01, self.simulation_callback)
        self.simulation = simulation.Simulation()

    def simulation_callback(self):
        self.simulation.step()

    def write_robot_state(self, request, response):
        a = 1

    def read_robot_state(self, request, response):
        # add filter like in stm32 bridge to get position a specific time
        a = 1


def main(args=None):
    rclpy.init(args=args)
    sim_node = Sim_Node()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()
