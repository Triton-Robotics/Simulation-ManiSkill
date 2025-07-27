from typing import Dict

import numpy as np
import sapien.physx as physx
import torch

from mani_skill.sensors.base_sensor import BaseSensor, BaseSensorConfig
from mani_skill.sensors.camera import Camera
from mani_skill.utils import common


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
