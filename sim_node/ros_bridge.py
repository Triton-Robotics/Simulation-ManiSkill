import time
import rclpy
from rclpy.node import Node
from tr_messages.srv import WriteSerial, ListenSerial
from sensor_msgs.msg import Image
from sim_node import simulation

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2  # pointcloud utilizes


class Sim_Node(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.write_service = self.create_service(
            WriteSerial, "write_robot_state", self.write_robot_state
        )
        self.listen_service = self.create_service(
            ListenSerial, "read_robot_state", self.read_robot_state
        )
        self.image_pub = self.create_publisher(PointCloud2, "pointcloud", 10)
        # 10ms
        self.simulation_timer = self.create_timer(0.01, self.simulation_callback)
        self.simulation = simulation.Simulation()

    def simulation_callback(self):
        obs = self.simulation.step()
        xyzw = obs["pointcloud"]["xyzw"]
        print("shape", xyzw.shape)
        xyzw = xyzw.squeeze(0)
        valid_mask = xyzw[:, 3] == 1
        points = xyzw[valid_mask, :3]
        msg = self.points_to_ros_pointcloud2(points)
        self.image_pub.publish(msg)

    def write_robot_state(self, request, response):
        a = 1

    def read_robot_state(self, request, response):
        # add filter like in stm32 bridge to get position a specific time
        a = 1

    def points_to_ros_pointcloud2(self, points):

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        field_x = PointField()
        field_x.name = "x"
        field_x.offset = 0
        field_x.datatype = PointField.FLOAT32
        field_x.count = 1

        # Do the same for 'y' and 'z':
        field_y = PointField()
        field_y.name = "y"
        field_y.offset = 4
        field_y.datatype = PointField.FLOAT32
        field_y.count = 1

        field_z = PointField()
        field_z.name = "z"
        field_z.offset = 8
        field_z.datatype = PointField.FLOAT32
        field_z.count = 1

        fields = [field_x, field_y, field_z]
        point_iter = points.tolist()

        pc2_msg = point_cloud2.create_cloud(header, fields, point_iter)
        return pc2_msg


def main(args=None):
    rclpy.init(args=args)
    sim_node = Sim_Node()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()
