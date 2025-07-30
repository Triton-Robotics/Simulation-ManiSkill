import time
import rclpy
from rclpy.node import Node
from tr_messages.srv import WriteSerial, ListenSerial
from tr_messages.msg import SimTeleopInput
from sensor_msgs.msg import Image
from sim_node import simulation

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2  # pointcloud utilizes
from sim_node import utils, constants
import numpy as np

from cv_bridge import CvBridge
import torch
from torch import Tensor


class Sim_Node(Node):
    def __init__(self):
        super().__init__("sim_node")

        # cv cam params
        self.declare_parameter("enable_cv_cam", True)
        self.declare_parameter("cv_resolution_x", 1920)
        self.declare_parameter("cv_resolution_y", 1200)
        self.declare_parameter("cv_fov_horizontal", 31)
        self.declare_parameter("cv_fov_vertical", 20)
        self.declare_parameter("cv_shader_pack", "default")
        # TODO add cv camera matrix parameter

        # lidar params
        self.declare_parameter("enable_lidar", True)
        self.declare_parameter("lidar_pointcloud_resolution", 20)

        # general simulation params
        self.declare_parameter("spawn_scenario", "center_1v1")
        self.declare_parameter("human_gui", True)

        self.primary_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/primary_robot_teleop",
            self.primary_robot_teleop_callback,
            10,
        )
        self.secondary_robot_teleop_sub = self.create_subscription(
            SimTeleopInput,
            "simulation/secondary_robot_teleop",
            self.secondary_robot_teleop_callback,
            10,
        )

        self.write_service = self.create_service(
            WriteSerial, "write_robot_state", self.write_robot_state
        )
        self.listen_service = self.create_service(
            ListenSerial, "read_robot_state", self.read_robot_state
        )
        self.pointcloud_pub = self.create_publisher(PointCloud2, "pointcloud", 10)
        qos_profile = rclpy.qos.qos_profile_sensor_data
        qos_profile.depth = 1
        self.image_pub = self.create_publisher(Image, "camera/image", qos_profile)
        # 10ms
        self.simulation_timer = self.create_timer(0.01, self.simulation_callback)
        options = dict(
            # general simulation options
            spawn_scenario=self.get_parameter("spawn_scenario")
            .get_parameter_value()
            .string_value,
            human_gui=self.get_parameter("human_gui").get_parameter_value().bool_value,
            primary_robot=dict(
                # cv cam options
                enable_cv_cam=self.get_parameter("enable_cv_cam")
                .get_parameter_value()
                .bool_value,
                cv_resolution_x=self.get_parameter("cv_resolution_x")
                .get_parameter_value()
                .integer_value,
                cv_resolution_y=self.get_parameter("cv_resolution_y")
                .get_parameter_value()
                .integer_value,
                cv_fov_horizontal=self.get_parameter("cv_fov_horizontal")
                .get_parameter_value()
                .integer_value,
                cv_fov_vertical=self.get_parameter("cv_fov_vertical")
                .get_parameter_value()
                .integer_value,
                cv_shader_pack=self.get_parameter("cv_shader_pack")
                .get_parameter_value()
                .string_value,
                # lidar options
                enable_lidar=self.get_parameter("enable_lidar")
                .get_parameter_value()
                .bool_value,
                lidar_pointcloud_resolution=self.get_parameter(
                    "lidar_pointcloud_resolution"
                )
                .get_parameter_value()
                .integer_value,
            ),
            # second robot is just used as target practice so we dont care about its sensors
            secondary_robot=dict(enable_cv_cam=False, enable_lidar=False),
        )

        self.simulation = simulation.Simulation(options=options)

        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state = utils.robot_state()
        self.teleop_desired_robot_state = utils.robot_state()
        self.secondary_robot_teleop_desired_state = utils.robot_state()
        self.last_recorded_robot_state = utils.robot_state()

        self.cv_bridge = CvBridge()

    def simulation_callback(self):
        start = time.time()
        t1 = time.time()

        primary_robot_state = None
        if self.control_mode == "programmatic":
            primary_robot_state = self.programmatic_desired_robot_state
        elif self.control_mode == "teleop":
            primary_robot_state = self.teleop_desired_robot_state
        else:
            RuntimeError(f"invalid control_mode. {self.control_mode}")

        obs = self.simulation.step(
            primary_robot_state=primary_robot_state,
            secondary_robot_state=self.secondary_robot_teleop_desired_state,
        )
        t2 = time.time()
        print("step sim: ", (t2 - t1) * 1000, "ms")

        robot_state_position: Tensor = obs["agent"]["infantry-0"]["qpos"]
        robot_state_position = robot_state_position.squeeze(0)  # remove batch dimension

        robot_state_velocity: Tensor = obs["agent"]["infantry-0"]["qvel"]
        robot_state_velocity = robot_state_velocity.squeeze(0)  # remove batch dimension

        self.last_recorded_robot_state = utils.robot_state(
            x_vel=robot_state_velocity[0].item(),
            y_vel=robot_state_velocity[1].item(),
            angular_vel=robot_state_velocity[2].item(),
            yaw=robot_state_position[3].item(),
            pitch=robot_state_position[4].item(),
        )

        if self.get_parameter("enable_cv_cam").get_parameter_value().bool_value:
            t1 = time.time()
            rgb_tensor = obs["sensor_data"]["cv_camera_0"]["rgb"]
            rgb_tensor: torch.Tensor
            rgb_tensor = rgb_tensor.squeeze(0)  # remove batch dimension

            rgb_array = rgb_tensor.numpy(force=True)
            img_msg = self.cv_bridge.cv2_to_imgmsg(rgb_array, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            t2 = time.time()
            print("cv process img: ", (t2 - t1) * 1000, "ms")

            t1 = time.time()
            self.image_pub.publish(img_msg)
            t2 = time.time()
            print("cv pub: ", (t2 - t1) * 1000, "ms")

        if self.get_parameter("enable_lidar").get_parameter_value().bool_value:
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
            self.pointcloud_pub.publish(msg)
            t2 = time.time()
            print("publishing pointcloud: ", (t2 - t1) * 1000, "ms")

        end = time.time()
        print("fps (theoretical): ", 1 / (end - start))
        print("time taken: ", (end - start) * 1000, "ms\n---\n")

    def read_robot_state(self, request, response):
        # TODO make a buffer and respond based off the time in the request
        # TOOD add angular vel and pitch and yaw vel
        response.x_vel = self.last_recorded_robot_state.x_vel
        response.y_vel = self.last_recorded_robot_state.y_vel
        response.pitch = self.last_recorded_robot_state.pitch
        response.yaw = self.last_recorded_robot_state.yaw
        response.pitch_vel = 0.0
        response.yaw_vel = 0.0
        response.success = True

        return response

    def write_robot_state(self, request, response):
        self.control_mode = "programmatic"
        self.programmatic_desired_robot_state = utils.robot_state(
            # pitch is negated so negative pitch means down
            -request.pitch,
            request.yaw,
            # we scale by max because maniskill normalizes velocity action space between -1,1
            request.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            request.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            request.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

        response.success = True
        return response

    # TODO fix this overriding the write_robot_state service call when all keys are let go of and 0's and being published
    def primary_robot_teleop_callback(self, msg):
        received_state = utils.robot_state(
            # pitch is negated so negative pitch means down
            pitch=-msg.pitch,
            yaw=msg.yaw,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

        if received_state != self.teleop_desired_robot_state:
            self.control_mode = "teleop"
            self.teleop_desired_robot_state = received_state

    def secondary_robot_teleop_callback(self, msg):
        self.secondary_robot_teleop_desired_state = utils.robot_state(
            pitch=0,
            yaw=0,
            x_vel=msg.x_vel / constants.MAX_TRANSLATION_VEL_M_S,
            y_vel=msg.y_vel / constants.MAX_TRANSLATION_VEL_M_S,
            angular_vel=msg.angular_vel / constants.MAX_ANGULAR_VEL_RADS_S,
        )

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
