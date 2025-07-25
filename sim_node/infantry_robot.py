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

    default_pos = sapien.Pose(p=[0, 0, 0.25], q=[1, 0, 0, 0])
    default_pos.set_rpy([np.deg2rad(90), 0, np.deg2rad(180)])

    keyframes = dict(
        default=Keyframe(pose=default_pos)
        # default=Keyframe(pose=sapien.Pose(p=[0, 0, 1.0], q=[1, 0, 0, 0]))
    )

    pitch_joint_names = ["pitch_joint"]
    yaw_joint_names = ["yaw_joint"]
    base_joint_names = [
        "root_x_axis_joint",
        "root_z_axis_joint",
        "root_y_rotation_joint",
    ]

    def _after_init(self):
        super()._after_init()

        # make light bars glow in rt rendering mode
        lightbar_link = self.robot.links_map["lightbars_link"]
        visual_block = lightbar_link.render_shapes[0]
        for shape in visual_block:
            for part in shape.parts:
                part.material.set_base_color([0, 0, 0.1, 1])
                part.material.set_emission([0, 0, 100, 100])
                part.material.set_emission_texture(None)
                part.material.set_metallic_texture(None)
                part.material.set_roughness_texture(None)

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
                # order matters. determines the order of the action space
                # this is set to match the URDF
                base=base_pd_joint_vel,
                yaw=yaw_pd_joint,
                pitch=pitch_pd_joint,
            )
        )

        return deepcopy_dict(controller_configs)

    @property
    def _sensor_configs(self):

        sensors = []
        # CV camera sensor
        sensors.append(
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
            )
        )

        # lidar simulated with multiple camera sensors
        lidar_pose = [0, 0.2, 0]
        pose0 = sapien.Pose(lidar_pose)
        # left cam
        pose0.set_rpy([0, np.deg2rad(-45), 0])

        # bottom cam
        pose1 = sapien.Pose(lidar_pose)
        pose1.set_rpy([0, np.deg2rad(-45), np.deg2rad(-90)])

        # right cam
        pose2 = sapien.Pose(lidar_pose)
        pose2.set_rpy([0, np.deg2rad(-45), np.deg2rad(-180)])

        # top cam
        pose3 = sapien.Pose(lidar_pose)
        pose3.set_rpy([0, np.deg2rad(-45), np.deg2rad(-270)])

        lidar_width_resolution = 200
        lidar_height_resolution = 200

        lidar_camera_intrinsics = np.array(
            [
                [0.5 * lidar_width_resolution, 0.0, 0.5 * lidar_width_resolution],
                [0.0, 0.5 * lidar_height_resolution, 0.5 * lidar_height_resolution],
                [0.0, 0.0, 1.0],
            ]
        )
        sensors.append(
            CameraConfig(
                uid="lidar_0",
                pose=pose0,
                width=lidar_width_resolution,
                height=lidar_height_resolution,
                intrinsic=lidar_camera_intrinsics,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            )
        )
        sensors.append(
            CameraConfig(
                uid="lidar_1",
                pose=pose1,
                width=lidar_width_resolution,
                height=lidar_height_resolution,
                intrinsic=lidar_camera_intrinsics,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            )
        )
        sensors.append(
            CameraConfig(
                uid="lidar_2",
                pose=pose2,
                width=lidar_width_resolution,
                height=lidar_height_resolution,
                intrinsic=lidar_camera_intrinsics,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            )
        )
        sensors.append(
            CameraConfig(
                uid="lidar_3",
                pose=pose3,
                width=lidar_width_resolution,
                height=lidar_height_resolution,
                intrinsic=lidar_camera_intrinsics,
                near=0.01,
                far=100,
                mount=self.robot.links_map["camera_link"],
                shader_pack="minimal",
            )
        )

        return sensors
