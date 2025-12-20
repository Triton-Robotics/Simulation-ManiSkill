from typing import Dict
import sapien

from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env
from ament_index_python import get_package_share_directory
import os
from sim_node import infantry_robot
import numpy as np
from sim_node import infantry_robot
from mani_skill.agents.multi_agent import MultiAgent
from mani_skill.sensors.camera import *
from mani_skill.sensors.depth_camera import StereoDepthCameraConfig, StereoDepthCamera
from mani_skill.sensors.camera import Camera
from mani_skill.utils.structs.render_camera import RenderCamera
from mani_skill.utils.structs.pose import Pose

from mani_skill.utils.structs.types import SimConfig
from sim_node.infantry_robot import InfantryRobot

package_dir = get_package_share_directory("sim_node")
base_field_path = package_dir + "/resource/models/field/"

floor_gltf = "2026_ARC_3v3_floor.gltf"
field_gltfs = [
    "2026_ARC_3v3_side_box.gltf",
    "2026_ARC_3v3_long_ramp_and_platform.gltf",
    "2026_ARC_3v3_platform_short_wall.gltf",
    "2026_ARC_3v3_outer_wall_top.gltf",
    "2026_ARC_3v3_outer_wall_right.gltf",
    "2026_ARC_3v3_outer_wall_bottom.gltf",
    "2026_ARC_3v3_outer_wall_left.gltf",
    # right side
    "2026_ARC_3v3_large_barrier_right.gltf",
    "2026_ARC_3v3_small_barrier_right.gltf",
    "2026_ARC_3v3_small_ramp_right.gltf",
    # left side
    "2026_ARC_3v3_large_barrier_left.gltf",
    "2026_ARC_3v3_small_barrier_left.gltf",
    "2026_ARC_3v3_small_ramp_left.gltf",
]


@register_env("ARC2026")
class ARC2026Env(BaseEnv):

    def __init__(self, *args, robot_uids=("infantry"), **kwargs):
        self.field_elements = []

        self.robot_keyframes = kwargs.pop("robot_keyframes")
        # cam stuff
        self.enable_cv_cam = kwargs.pop("enable_cv_cam")
        self.cv_exposure = kwargs.pop("cv_exposure")
        self.cv_resolution_x = kwargs.pop("cv_resolution_x")
        self.cv_resolution_y = kwargs.pop("cv_resolution_y")
        self.cv_fov_horizontal = kwargs.pop("cv_fov_horizontal")
        self.cv_fov_vertical = kwargs.pop("cv_fov_vertical")
        self.cv_ray_tracing = kwargs.pop("cv_ray_tracing")
        # lidar
        self.enable_lidar = kwargs.pop("enable_lidar")
        self.lidar_pointcloud_resolution = kwargs.pop("lidar_pointcloud_resolution")

        # TODO one day fix parallel in single scene and how it breaks camera sensors on multi agent
        # self._parallel_in_single_scene = kwargs.get("parallel_in_single_scene", False)

        kwargs["sim_config"] = SimConfig(spacing=15)
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    def _get_obs_extra(self, info: Dict):
        return dict(
            # TODO there is probably a better way to get dt in maniskill
            sim_timestamp=(info["elapsed_steps"][0] * self.control_timestep).item(),
            primary_robot=self.agent.agents[0].get_ground_truth_obs(),
            secondary_robot=self.agent.agents[1].get_ground_truth_obs(),
        )

    def _load_agent(self, options: dict):
        primary_agent = InfantryRobot(
            scene=self.scene,
            control_freq=self._control_freq,
            control_mode=self._control_mode,
            agent_idx=0,
            initial_pose=sapien.Pose(p=[10, 10, 4]),
            # build_separate=True,
            enable_cv_cam=self.enable_cv_cam,
            cv_ray_tracing=self.cv_ray_tracing,
            cv_resolution_x=self.cv_resolution_x,
            cv_resolution_y=self.cv_resolution_y,
            cv_fov_horizontal=self.cv_fov_horizontal,
            cv_fov_vertical=self.cv_fov_vertical,
            enable_lidar=self.enable_lidar,
            lidar_pointcloud_resolution=self.lidar_pointcloud_resolution,
            keyframe=self.robot_keyframes[0],
        )

        secondary_agent = infantry_robot.InfantryRobot(
            scene=self.scene,
            control_freq=self._control_freq,
            control_mode=self._control_mode,
            agent_idx=1,
            initial_pose=sapien.Pose(p=[10, 10, 2]),
            keyframe=self.robot_keyframes[1],
        )

        self.agent = MultiAgent(agents=[primary_agent, secondary_agent])

    def _load_lighting(self, options: dict):
        # self.scene.set_ambient_light([0.05, 0.05, 0.05])
        return super()._load_lighting(options)

    # This is mostly copied directly from maniskill source code
    def _setup_sensors(self, options: dict):
        # First create all the configurations
        self._sensor_configs = dict()

        # Add task/external sensors
        self._sensor_configs.update(parse_camera_configs(self._default_sensor_configs))

        # Add agent sensors
        self._agent_sensor_configs = dict()
        if self.agent is not None:
            self.agent: MultiAgent
            # !!!! IMPORTANT !!!!
            # Fixed problem with multi agent wrapper here
            # need to use agent.sensor_configs not agent._sensor_configs
            self._agent_sensor_configs = parse_camera_configs(self.agent.sensor_configs)
            self._sensor_configs.update(self._agent_sensor_configs)

        # Add human render camera configs
        self._human_render_camera_configs = parse_camera_configs(
            self._default_human_render_camera_configs
        )

        self._viewer_camera_config = parse_camera_configs(
            self._default_viewer_camera_configs
        )

        # Override camera configurations with user supplied configurations
        if self._custom_sensor_configs is not None:
            update_camera_configs_from_dict(
                self._sensor_configs, self._custom_sensor_configs
            )
        if self._custom_human_render_camera_configs is not None:
            update_camera_configs_from_dict(
                self._human_render_camera_configs,
                self._custom_human_render_camera_configs,
            )
        if self._custom_viewer_camera_configs is not None:
            update_camera_configs_from_dict(
                self._viewer_camera_config,
                self._custom_viewer_camera_configs,
            )
        self._viewer_camera_config = self._viewer_camera_config["viewer"]

        # Now we instantiate the actual sensor objects
        self._sensors = dict()

        for uid, sensor_config in self._sensor_configs.items():
            if uid in self._agent_sensor_configs:
                # TODO this means only our primary robot can have sensors. Use the index in uid to determine specific agents
                articulation = self.agent.agents[0].robot
            else:
                articulation = None
            if isinstance(sensor_config, StereoDepthCameraConfig):
                sensor_cls = StereoDepthCamera
            elif isinstance(sensor_config, CameraConfig):
                sensor_cls = Camera
            self._sensors[uid] = sensor_cls(
                sensor_config,
                self.scene,
                articulation=articulation,
            )

        # Cameras for rendering only
        self._human_render_cameras = dict()
        for uid, camera_config in self._human_render_camera_configs.items():
            self._human_render_cameras[uid] = Camera(
                camera_config,
                self.scene,
            )

        self.scene.sensors = self._sensors
        self.scene.human_render_cameras = self._human_render_cameras

    def _initialize_episode(self, env_idx, options):
        for e in self.field_elements:
            e.set_pose(sapien.Pose(p=[0, 0, 0], q=e.pose[0].sp.q))
        # TODO: it might be possible to randomize field element positions, however there are issues with it rn
        # for e in self.field_elements:
        #     p = torch.rand((3))
        #     q = torch.rand((4)) / 100
        #     e.set_pose(sapien.Pose(p=p, q=q))

    def _load_scene(self, options: dict):
        floor_visual_builder = self.scene.create_actor_builder()
        floor_visual_builder.add_visual_from_file(filename=base_field_path + floor_gltf)
        floor_visual_builder.set_initial_pose(sapien.Pose(p=[0, 0, 1]))
        floor_obj = floor_visual_builder.build_kinematic(name="floor")
        self.field_elements.append(floor_obj)

        for i, name in enumerate(field_gltfs):
            field_element_builder = self.scene.create_actor_builder()

            field_element_builder.add_visual_from_file(filename=base_field_path + name)

            # TODO add ros param to turn on / off collisions. Would help speed up initial boot time
            field_element_builder.add_multiple_convex_collisions_from_file(
                filename=base_field_path + name,
                # TODO fix coacd not working with venv
                decomposition="coacd",
                decomposition_params=dict(
                    threshold=1.0,
                    preprocess_mode="off",
                ),
            )

            # prevents collisions on startup of environment
            pos = [0, 0, 3 * (i + 1)]
            field_element_builder.set_initial_pose(sapien.Pose(p=pos))
            field_element = field_element_builder.build_kinematic(name)
            self.field_elements.append(field_element)

    def _after_reconfigure(self, options):
        super()._after_reconfigure(options)

        for name, sensor in self._sensors.items():
            if "cv_camera" in name:
                sensor: Camera
                render_cam: RenderCamera = sensor.camera
                render_cam.set_property("exposure", self.cv_exposure)
