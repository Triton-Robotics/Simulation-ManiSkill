from typing import Dict

import numpy as np
import sapien.physx as physx
import torch

from mani_skill.sensors.base_sensor import BaseSensor, BaseSensorConfig
from mani_skill.sensors.camera import Camera
from mani_skill.utils import common

from tr_messages.msg import RobotGroundTruth, SimGroundTruth
from geometry_msgs.msg import Pose, Vector3


class robot_state:
    def __init__(
        self,
        pitch: float = 0,
        yaw: float = 0,
        x_vel: float = 0,
        y_vel: float = 0,
        angular_vel: float = 0,
    ) -> None:
        self.pitch = pitch
        self.yaw = yaw
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.angular_vel = angular_vel

    def __eq__(self, value: object) -> bool:
        return (
            self.pitch == value.pitch
            and self.yaw == value.yaw
            and self.x_vel == value.x_vel
            and self.y_vel == value.y_vel
            and self.angular_vel == value.angular_vel
        )

    def __str__(self) -> str:
        return (
            f"{self.x_vel}, {self.y_vel}, {self.angular_vel}, {self.pitch}, {self.yaw}"
        )


def sensor_data_to_pointcloud(observation: Dict):
    """convert all camera data in sensor to pointcloud data"""
    sensor_data = observation["sensor_data"]
    camera_params = observation["sensor_param"]
    pointcloud_obs = dict()
    # print(sensor_data.items())

    for cam_uid, images in sensor_data.items():
        if "lidar" in cam_uid:
            cam_pcd = {}
            # TODO: double check if the .clone()s are necessary
            # Each pixel is (x, y, z, actor_id) in OpenGL camera space
            # actor_id = 0 for the background
            images: Dict[str, torch.Tensor]
            position = images["position"].clone()
            segmentation = images["segmentation"].clone()
            position = position.float()
            position[..., :3] = (
                position[..., :3] / 1000.0
            )  # convert the raw depth from millimeters to meters

            # Convert to world space
            cam2world = camera_params[cam_uid]["cam2world_gl"].to(position.device)
            xyzw = torch.cat([position, segmentation != 0], dim=-1).reshape(
                position.shape[0], -1, 4
            ) @ cam2world.transpose(1, 2)
            cam_pcd["xyzw"] = xyzw

            # Extra keys
            if "rgb" in images:
                rgb = images["rgb"][..., :3].clone()
                cam_pcd["rgb"] = rgb.reshape(rgb.shape[0], -1, 3)
            if "segmentation" in images:
                cam_pcd["segmentation"] = segmentation.reshape(
                    segmentation.shape[0], -1, 1
                )

            pointcloud_obs[cam_uid] = cam_pcd

    pointcloud_obs = common.merge_dicts(pointcloud_obs.values())
    for key, value in pointcloud_obs.items():
        pointcloud_obs[key] = torch.concat(value, axis=1)

    return pointcloud_obs


# obs should be the dict of the specific robot from extra obs that this msg belongs to
# ex: obs = obs["extra"]["primary_robot"]
def populate_robot_ground_truth_msg(msg: RobotGroundTruth, obs: dict) -> None:
    populate_pose_msg_from_list(msg.chassis_pose, obs["chassis_pose"])
    populate_pose_msg_from_list(msg.turret_pose, obs["turret_pose"])
    populate_pose_msg_from_list(msg.camera_pose, obs["camera_pose"])
    populate_pose_msg_from_list(msg.lidar_pose, obs["lidar_pose"])

    msg.armor_panel_poses = []
    for panel in obs["panel_poses"]:
        p = Pose()
        populate_pose_msg_from_list(p, panel)
        msg.armor_panel_poses.append(p)

    msg.chassis_imu.orientation.w = 1.0
    msg.chassis_imu.orientation.x = 0.0
    msg.chassis_imu.orientation.y = 0.0
    msg.chassis_imu.orientation.z = 0.0

    angular_vel = Vector3()
    angular_vel.x = 0.0
    angular_vel.y = 0.0
    angular_vel.z = obs["lidar_imu"][2]
    msg.chassis_imu.angular_velocity = angular_vel

    linear_accel = Vector3()
    linear_accel.x = obs["lidar_imu"][0]
    linear_accel.y = obs["lidar_imu"][1]
    linear_accel.z = 0.0
    msg.chassis_imu.linear_acceleration = linear_accel


def populate_pose_msg_from_list(msg: Pose, pose_list: list) -> None:
    msg.position.x = pose_list[0]
    msg.position.y = pose_list[1]
    msg.position.z = pose_list[2]

    msg.orientation.w = pose_list[3]
    msg.orientation.x = pose_list[4]
    msg.orientation.y = pose_list[5]
    msg.orientation.z = pose_list[6]
