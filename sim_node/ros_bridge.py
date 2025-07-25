import time
import rclpy
from rclpy.node import Node
from tr_messages.srv import WriteSerial, ListenSerial
from sensor_msgs.msg import Image
from sim_node import simulation

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2  # pointcloud utilizes
from sim_node import utils
import numpy as np

# TODO:
# add launch files for camera and lidar scenario for sim
# add lidar support
# write and listen services working with the action space
# write keyboard control node that calls write service


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

        self.desired_robot_state = utils.robot_state()

    def simulation_callback(self):
        start = time.time()
        t1 = time.time()
        obs = self.simulation.step(self.desired_robot_state)
        t2 = time.time()
        print("step sim: ", (t2 - t1) * 1000, "ms")

        # Todo fix crash for when no lidar cameras are defined
        t1 = time.time()
        pointcloud = utils.sensor_data_to_pointcloud(obs)
        t2 = time.time()
        print("raw to pointcloud: ", (t2 - t1) * 1000, "ms")

        t1 = time.time()
        xyzw = pointcloud["xyzw"]
        xyzw = xyzw.squeeze(0)
        valid_mask = xyzw[:, 3] == 1
        points = xyzw[valid_mask, :3]
        msg = self.points_to_ros_pointcloud2(points)
        t2 = time.time()
        print("pointcloud to ros: ", (t2 - t1) * 1000, "ms")

        t1 = time.time()
        self.image_pub.publish(msg)
        t2 = time.time()
        print("publishing: ", (t2 - t1) * 1000, "ms")

        end = time.time()
        print("fps (theoretical): ", 1 / (end - start))
        print("time taken: ", (end - start) * 1000, "ms\n---\n")

    def write_robot_state(self, request, response):
        self.desired_robot_state = utils.robot_state(
            request.pitch, request.yaw, request.x_vel, request.y_vel
        )

        response.success = True
        return response

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
        point_iter = points.cpu().numpy()

        pc2_msg = point_cloud2.create_cloud(header, fields, point_iter)
        return pc2_msg


def main(args=None):
    rclpy.init(args=args)
    sim_node = Sim_Node()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()
