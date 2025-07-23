import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig
from ament_index_python import get_package_share_directory
import os

package_dir = get_package_share_directory("sim_node")


@register_agent()
class InfantryRobot(BaseAgent):
    uid = "infantry"
    urdf_path = str(
        os.path.join(package_dir, "resource/models/infantry/infantry-blue.urdf")
    )
    keyframes = dict(
        default=Keyframe(pose=sapien.Pose(p=[1, 1, 0.25], q=[0.707, 0.707, 0, 0]))
        # default=Keyframe(pose=sapien.Pose(p=[0, 0, 1.0], q=[1, 0, 0, 0]))
    )

    pitch_joint_names = ["pitch_joint"]
    yaw_joint_names = ["yaw_joint"]
    base_joint_names = [
        "root_x_axis_joint",
        "root_z_axis_joint",
        "root_y_rotation_joint",
    ]

    @property
    def _controller_configs(self):

        pitch_pd_joint = PDJointPosControllerConfig(
            self.pitch_joint_names,
            lower=None,
            upper=None,
            stiffness=1000,
            damping=100,
            force_limit=100,
            normalize_action=False,
        )

        yaw_pd_joint = PDJointPosControllerConfig(
            self.yaw_joint_names,
            lower=None,
            upper=None,
            stiffness=1000,
            damping=100,
            force_limit=100,
            normalize_action=False,
        )

        base_pd_joint_vel = PDBaseVelControllerConfig(
            self.base_joint_names,
            lower=[-1, -1, -3.14],
            upper=[1, 1, 3.14],
            damping=1000,
            force_limit=500,
        )

        controller_configs = dict(
            pd_standard=dict(
                pitch=pitch_pd_joint, yaw=yaw_pd_joint, base=base_pd_joint_vel
            )
        )

        return deepcopy_dict(controller_configs)

    @property
    def _sensor_configs(self):
        q_test = [0.6533, -0.2706, -0.6533, -0.2706]
        pose = sapien.Pose(p=[0, 0.2, 0], q=[1, 0, 0, 0])
        dtr = 3.1415 / 180.0
        pose.set_rpy([0, -45 * dtr, 0])

        pose2 = sapien.Pose(p=[0, 0.2, 0], q=[1, 0, 0, 0])
        pose2.set_rpy([0, -45 * dtr, -90 * dtr])

        pose3 = sapien.Pose(p=[0, 0.2, 0], q=[1, 0, 0, 0])
        pose3.set_rpy([0, -45 * dtr, -180 * dtr])

        pose4 = sapien.Pose(p=[0, 0.2, 0], q=[1, 0, 0, 0])
        pose4.set_rpy([0, -45 * dtr, -270 * dtr])

        width = 200
        height = 200

        instincts = np.array(
            [
                [0.5 * width, 0.0, 0.5 * width],
                [0.0, 0.5 * height, 0.5 * height],
                [0.0, 0.0, 1.0],
            ]
        )
        return [
            CameraConfig(
                uid="cv_camera",
                pose=sapien.Pose(p=[0, 0, 0], q=[0.5, -0.5, -0.5, -0.5]),
                width=1920,
                height=1200,
                fov=90,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            ),
            CameraConfig(
                uid="lidar_1",
                pose=pose,
                width=200,
                height=200,
                intrinsic=instincts,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            ),
            CameraConfig(
                uid="lidar_2",
                pose=pose2,
                width=200,
                height=200,
                intrinsic=instincts,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            ),
            CameraConfig(
                uid="lidar_3",
                pose=pose3,
                width=200,
                height=200,
                intrinsic=instincts,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            ),
            # CameraConfig(
            #     uid="lidar_4",
            #     pose=pose4,
            #     width=200,
            #     height=200,
            #     intrinsic=instincts,
            #     near=0.01,
            #     far=100,
            #     mount=self.robot.links_map["camera_link"],
            #     shader_pack="minimal",
            # ),
        ]
