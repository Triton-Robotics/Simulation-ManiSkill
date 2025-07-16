import sapien

from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env
from ament_index_python import get_package_share_directory
import os
from sim_node import infantry_robot

package_dir = get_package_share_directory("sim_node")
urdf_path = os.path.join(package_dir, "resource/models/arena/arena.urdf")


@register_env("comp_field")
class CompFieldEnv(BaseEnv):

    def __init__(self, *args, robot_uids=("infantry"), **kwargs):
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    def _load_agent(self, options: dict):
        # set a reasonable initial pose for the agent that doesn't intersect other objects
        options
        super()._load_agent(
            options, [sapien.Pose(p=[1, 1, 1]), sapien.Pose(p=[1, 5, 3])]
        )

    def _load_scene(self, options: dict):
        loader = self.scene.create_urdf_loader()
        actor_builders = loader.parse(str(urdf_path))["actor_builders"]
        # print(builders)
        builder = actor_builders[0]

        builder.initial_pose = sapien.Pose(p=[0, 0, 0])
        builder.set_physx_body_type("static")
        builder.build(name="comp_field")
