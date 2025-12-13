import gymnasium as gym
import mani_skill.envs
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.agents.base_agent import BaseAgent
from mani_skill.agents.multi_agent import MultiAgent
from sim_node import ARC2026_env
import numpy as np
from sim_node import utils
from mani_skill.utils.structs import SimConfig
import torch

SPAWN_SCENARIO_KEYFRAME_MAPPING: dict = dict(
    center_1v1=dict(
        primary_robot="default",
        secondary_robot="aiming_target",
    ),
    navigate_from_spawn=dict(
        primary_robot="blue_spawn",
        secondary_robot="out_of_field",
    ),
)


class Simulation:

    def __init__(self, seed=2930, **kwargs):
        self.should_render_gui = kwargs.pop("human_gui")
        cpu_sim = kwargs.pop("cpu_sim")

        sim_config = SimConfig()
        SimConfig.control_freq = kwargs.pop("control_freq")
        sim_config.sim_freq = kwargs.pop("sim_freq")

        self.env: BaseEnv = gym.make(
            "ARC2026",
            render_mode=("human" if self.should_render_gui else None),
            reward_mode="sparse",
            obs_mode="state_dict+rgb+segmentation+position",
            sim_config=sim_config,
            sim_backend="physx_cpu" if cpu_sim else "auto",
            render_backend="sapien_cpu" if cpu_sim else "sapien_cuda",
            # num_envs=2,
            # parallel_in_single_scene=True,
            **kwargs,
        )

        self.base_env: BaseEnv = self.env
        # TODO dont make this none make it the initial reset obs instead
        self.past_obs = None

        obs, _ = self.env.reset(seed=seed)

    def step(
        self,
        primary_robot_state: utils.robot_state,
        secondary_robot_state: utils.robot_state,
    ):
        # calculate world relative robot velocity
        if self.past_obs is not None:
            primary_world_relative_vel = self.head_to_world_vel(
                "infantry-0",
                primary_robot_state.x_vel,
                primary_robot_state.y_vel,
            )
            secondary_world_relative_vel = self.head_to_world_vel(
                "infantry-1",
                secondary_robot_state.x_vel,
                secondary_robot_state.y_vel,
            )
        else:
            primary_world_relative_vel = np.array([0, 0])
            secondary_world_relative_vel = np.array([0, 0])

        primary_robot_action = torch.tensor(
            [
                primary_world_relative_vel[0],
                primary_world_relative_vel[1],
                primary_robot_state.angular_vel,
                primary_robot_state.yaw,
                primary_robot_state.pitch,
            ],
            dtype=torch.float32,
        )
        secondary_robot_action = torch.tensor(
            [
                secondary_world_relative_vel[0],
                secondary_world_relative_vel[1],
                secondary_robot_state.angular_vel,
                secondary_robot_state.yaw,
                secondary_robot_state.pitch,
            ],
            dtype=torch.float32,
        )

        primary_robot_action_batched = primary_robot_action.unsqueeze(0).expand(
            self.env.unwrapped.num_envs, -1
        )
        secondary_robot_action_batched = secondary_robot_action.unsqueeze(0).expand(
            self.env.unwrapped.num_envs, -1
        )

        action = {
            "infantry-0": primary_robot_action_batched,
            "infantry-1": secondary_robot_action_batched,
        }

        obs, reward, terminated, truncated, info = self.env.step(action=action)
        if self.should_render_gui:
            self.env.render()

        self.past_obs = obs
        return obs

    def shutdown(self):
        self.env.close()

    def head_to_world_vel(self, robot: str, x_vel, y_vel):
        yaw_rads = self.past_obs["agent"][robot]["qpos"][0][3].item()
        rotation_matrix = np.array(
            [
                [np.cos(yaw_rads), -np.sin(yaw_rads)],
                [np.sin(yaw_rads), np.cos(yaw_rads)],
            ]
        )

        head_relative_vel = np.array([x_vel, y_vel])
        world_relative_vel = rotation_matrix @ head_relative_vel

        return world_relative_vel
