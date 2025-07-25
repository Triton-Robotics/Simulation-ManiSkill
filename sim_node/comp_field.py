import sapien

from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env
from ament_index_python import get_package_share_directory
import os
from sim_node import infantry_robot
import numpy as np

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
        # set a reasonable initial pose for the agent that doesn't intersect other objects
        super()._load_agent(
            options, [sapien.Pose(p=[1, 1, 1]), sapien.Pose(p=[1, 5, 3])]
        )

    def _load_lighting(self, options: dict):
        # self.scene.set_ambient_light([0.05, 0.05, 0.05])
        return super()._load_lighting(options)

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
