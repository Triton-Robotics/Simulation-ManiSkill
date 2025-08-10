from mani_skill.envs.scene import ManiSkillScene
from mani_skill.utils.structs.pose import Pose
import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig
from torch import Tensor
from ament_index_python import get_package_share_directory
import os
from sim_node import constants
from mani_skill.render.shaders import ShaderConfig, default_position_texture_transform
import torch

package_dir = get_package_share_directory("sim_node")


@register_agent()
class InfantryRobot(BaseAgent):
    uid = "infantry"
    urdf_path = str(
        os.path.join(package_dir, "resource/models/infantry/infantry-blue.urdf")
    )

    # TODO ideally we define a srdf file instead of disabling all collisions. That way we only disable problematic collisions and
    disable_self_collisions = True
    load_multiple_collisions = True

    def __init__(
        self,
        scene: ManiSkillScene,
        control_freq: int,
        control_mode: str | None = None,
        agent_idx: str | None = None,
        initial_pose: Pose | Pose | None = None,
        build_separate: bool = False,
        options: dict = [],
    ):
        super().__init__(
            scene, control_freq, control_mode, agent_idx, initial_pose, build_separate
        )
        self.options = options

    default_pos = sapien.Pose(p=[0, 0, 0.25], q=[1, 0, 0, 0])
    default_pos.set_rpy([np.deg2rad(90), 0, np.deg2rad(45)])

    aiming_target_pos = sapien.Pose(p=[2, -3, 0.25], q=[1, 0, 0, 0])
    aiming_target_pos.set_rpy([np.deg2rad(90), 0, np.deg2rad(180)])

    keyframes = dict(
        default=Keyframe(pose=default_pos),
        aiming_target=Keyframe(pose=aiming_target_pos),
    )

    pitch_joint_names = ["pitch_joint"]
    yaw_joint_names = ["yaw_joint"]
    base_joint_names = [
        # weird ordering because y axis up cadd
        "root_z_axis_joint",
        "root_x_axis_joint",
        "root_y_rotation_joint",
    ]

    def reset(self, init_qpos: Tensor = None):
        super().reset(init_qpos)

        if "keyframe" in self.options:
            desired_keyframe = self.options["keyframe"]
            keyframe = self.keyframes[desired_keyframe]
            self.robot.set_pose(keyframe.pose)

    def _after_init(self):
        super()._after_init()

    def _after_loading_articulation(self):
        super()._after_loading_articulation()

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

        # TODO use PDJointPosVelController for yaw
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
            lower=[
                -constants.MAX_TRANSLATION_VEL_M_S,
                -constants.MAX_TRANSLATION_VEL_M_S,
                -constants.MAX_ANGULAR_VEL_RADS_S,
            ],
            upper=[
                constants.MAX_TRANSLATION_VEL_M_S,
                constants.MAX_TRANSLATION_VEL_M_S,
                constants.MAX_ANGULAR_VEL_RADS_S,
            ],
            damping=1000,
            force_limit=500,
        )

        controller_configs = dict(
            pd_standard=dict(
                # order matters. determines the order of the action space
                # this is set to match the URDF
                # x, y, angular, yaw, pitch
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
        if self.options.get("enable_cv_cam", False):
            width = self.options["cv_resolution_x"]
            height = self.options["cv_resolution_y"]
            horizontal_fov = self.options["cv_fov_horizontal"]
            vertical_fov = self.options["cv_fov_vertical"]
            f_x = width / (2 * np.tan(np.radians(horizontal_fov) / 2))
            f_y = height / (2 * np.tan(np.radians(vertical_fov) / 2))
            c_x = width / 2
            c_y = height / 2
            cv_camera_intrinsics = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])

            # taken from Shaders.py in maniskill repo
            cv_texture_names = {
                "Color": ["rgb"],
                "Segmentation": ["segmentation"],
            }

            cv_texture_transforms = {
                "Color": lambda data: {"rgb": (data[..., :3] * 255).to(torch.uint8)},
                "Segmentation": lambda data: {"segmentation": data[..., 1][..., None]},
            }

            # TODO make a boolean raytracing rosparam to toggle between rt and non rt configs
            cv_shader_config_rt = ShaderConfig(
                shader_pack="rt",
                texture_names=cv_texture_names,
                shader_pack_config={
                    "ray_tracing_samples_per_pixel": 2,
                    "ray_tracing_path_depth": 1,
                    "ray_tracing_denoiser": "optix",
                },
                texture_transforms=cv_texture_transforms,
            )

            cv_shader_config_raster = ShaderConfig(
                shader_pack="default",
                texture_names=cv_texture_names,
                texture_transforms=cv_texture_transforms,
            )

            sensors.append(
                CameraConfig(
                    uid="cv_camera_" + str(self._agent_idx),
                    pose=sapien.Pose(p=[0, 0, 0], q=[1, 0, 0, 0]),
                    width=width,
                    height=height,
                    intrinsic=cv_camera_intrinsics,
                    near=0.01,
                    far=100,
                    entity_uid="camera_link",
                    shader_config=(
                        cv_shader_config_rt
                        if self.options["cv_ray_tracing"]
                        else cv_shader_config_raster
                    ),
                )
            )

        if self.options.get("enable_lidar", False):
            # lidar simulated with multiple camera sensors
            pose0 = sapien.Pose()
            # left cam
            pose0.set_rpy([0, np.deg2rad(-45), np.deg2rad(0)])

            # bottom cam
            pose1 = sapien.Pose()
            pose1.set_rpy([0, np.deg2rad(-45), np.deg2rad(90)])

            # right cam
            pose2 = sapien.Pose()
            pose2.set_rpy([0, np.deg2rad(-45), np.deg2rad(180)])

            # top cam
            pose3 = sapien.Pose()
            pose3.set_rpy([0, np.deg2rad(-45), np.deg2rad(270)])

            lidar_width_resolution = self.options["lidar_pointcloud_resolution"]
            lidar_height_resolution = self.options["lidar_pointcloud_resolution"]

            lidar_camera_intrinsics = np.array(
                [
                    [0.5 * lidar_width_resolution, 0.0, 0.5 * lidar_width_resolution],
                    [0.0, 0.5 * lidar_height_resolution, 0.5 * lidar_height_resolution],
                    [0.0, 0.0, 1.0],
                ]
            )

            lidar_texture_names = {
                "Position": ["position", "depth"],
                "Segmentation": ["segmentation"],
            }

            lidar_texture_transforms = {
                "Position": default_position_texture_transform,
                # note in default shader pack, 0 is visual shape / mesh, 1 is actor/link level, 2 is parallel scene ID, 3 is unused
                "Segmentation": lambda data: {"segmentation": data[..., 1][..., None]},
            }

            lidar_shader_config = ShaderConfig(
                shader_pack="default",
                texture_names=lidar_texture_names,
                shader_pack_config={},
                texture_transforms=lidar_texture_transforms,
            )

            sensors.append(
                CameraConfig(
                    uid="lidar_0_" + str(self._agent_idx),
                    pose=pose0,
                    width=lidar_width_resolution,
                    height=lidar_height_resolution,
                    intrinsic=lidar_camera_intrinsics,
                    near=0.01,
                    far=100,
                    entity_uid="lidar_link",
                    shader_config=lidar_shader_config,
                )
            )
            sensors.append(
                CameraConfig(
                    uid="lidar_1_" + str(self._agent_idx),
                    pose=pose1,
                    width=lidar_width_resolution,
                    height=lidar_height_resolution,
                    intrinsic=lidar_camera_intrinsics,
                    near=0.01,
                    far=100,
                    entity_uid="lidar_link",
                    shader_config=lidar_shader_config,
                )
            )
            sensors.append(
                CameraConfig(
                    uid="lidar_2_" + str(self._agent_idx),
                    pose=pose2,
                    width=lidar_width_resolution,
                    height=lidar_height_resolution,
                    intrinsic=lidar_camera_intrinsics,
                    near=0.01,
                    far=100,
                    entity_uid="lidar_link",
                    shader_config=lidar_shader_config,
                )
            )
            sensors.append(
                CameraConfig(
                    uid="lidar_3_" + str(self._agent_idx),
                    pose=pose3,
                    width=lidar_width_resolution,
                    height=lidar_height_resolution,
                    intrinsic=lidar_camera_intrinsics,
                    near=0.01,
                    far=100,
                    entity_uid="lidar_link",
                    shader_config=lidar_shader_config,
                )
            )

        return sensors
