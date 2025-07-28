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

package_dir = get_package_share_directory("sim_node")
base_field_path = "resource/models/field/"

full_field_visual_gltf_path = package_dir + "/resource/models/field.gltf"

urdf_path_and_names = [
    (
        "FieldSideLongWall",
        base_field_path + "FieldSideLongWall/FieldSideLongWall.urdf",
        True,
    ),
    (
        "FieldSideShortWall",
        base_field_path + "FieldSideShortWall/FieldSideShortWall.urdf",
        True,
    ),
    ("Floor", base_field_path + "Floor/Floor.urdf", False),
    ("LargeBarrier", base_field_path + "LargeBarrier/LargeBarrier.urdf", True),
    ("MiddleBarrier", base_field_path + "MiddleBarrier/MiddleBarrier.urdf", True),
    ("Ramp", base_field_path + "Ramp/Ramp.urdf", True),
    ("SmallPlatform", base_field_path + "SmallPlatform/SmallPlatform.urdf", True),
    ("BigPlatform", base_field_path + "BigPlatform/BigPlatform.urdf", True),
]


@register_env("comp_field")
class CompFieldEnv(BaseEnv):

    def __init__(self, *args, robot_uids=("infantry"), **kwargs):
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    def _load_agent(self, options: dict):
        user_options = options.get("user", dict())
        primary_agent = infantry_robot.InfantryRobot(
            scene=self.scene,
            control_freq=self._control_freq,
            control_mode=self._control_mode,
            agent_idx=0,
            initial_pose=sapien.Pose(p=[2, 1, 0]),
            build_separate=True,
            options=user_options.get("primary_robot", dict()),
        )

        secondary_agent = infantry_robot.InfantryRobot(
            scene=self.scene,
            control_freq=self._control_freq,
            control_mode=self._control_mode,
            agent_idx=1,
            initial_pose=sapien.Pose(p=[1, 1, 0]),
            build_separate=True,
            options=user_options.get("secondary_robot", dict()),
        )

        self.agent = MultiAgent(agents=[primary_agent, secondary_agent])

    def _load_lighting(self, options: dict):
        # self.scene.set_ambient_light([0.05, 0.05, 0.05])
        return super()._load_lighting(options)

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

    def _load_scene(self, options: dict):
        field_visual_builder = self.scene.create_actor_builder()

        field_visual_builder.add_visual_from_file(filename=full_field_visual_gltf_path)
        field_visual_builder.initial_pose = sapien.Pose()
        field_visual_builder.build_static(name="full_field_visual")

        loader = self.scene.create_urdf_loader()
        for name, path, load_flipped_copy in urdf_path_and_names:
            full_urdf_path = os.path.join(package_dir, path)
            actor_builders = loader.parse(str(full_urdf_path))["actor_builders"]
            builder = actor_builders[0]
            builder.set_physx_body_type("static")
            builder.initial_pose = sapien.Pose(p=[0, 0, 0])
            origional_actor = builder.build(name=name)

            if load_flipped_copy:
                actor_builders = loader.parse(str(full_urdf_path))["actor_builders"]
                builder = actor_builders[0]
                builder.set_physx_body_type("static")

                # position is defined in the STL. So when we pass a quaternion that is a 180 rotation
                # about the z axis it rotates the object around the origin (defined in the STL)
                # which correctly positions and orients the object
                flipped_pos = [0, 0, 0]
                q_rotate_180_z_axis = [0, 0, 0, 1]
                builder.initial_pose = sapien.Pose(p=flipped_pos, q=q_rotate_180_z_axis)
                builder.build(name=(name + "_flipped"))
