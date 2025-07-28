import gymnasium as gym
import mani_skill.envs
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.agents.base_agent import BaseAgent
from mani_skill.agents.multi_agent import MultiAgent
from sim_node import comp_field
import numpy as np
from sim_node import utils


class Simulation:

    def __init__(self, options: dict, seed=2930):
        self.env: BaseEnv = gym.make(
            "comp_field",  # This should map to your registered environment
            render_mode="human",  # Use "human" to visualize, or "rgb_array" to render frames without GUI
            reward_mode="sparse",
            obs_mode="state_dict+rgb+segmentation+position",
        )
        self.options = dict(reconfigure=True, user=options)

        self.base_env: BaseEnv = self.env
        self.past_obs = None

        obs, _ = self.env.reset(seed=seed, options=self.options)

        multi_agent: MultiAgent = self.env.get_wrapper_attr("agent")
        primary_agent: BaseAgent = multi_agent.agents[0]
        secondary_agent: BaseAgent = multi_agent.agents[1]
        primary_agent.robot.set_pose(primary_agent.keyframes["default"].pose)
        # secondary_agent.robot.set_pose(secondary_agent.keyframes["default"])

    def step(self, desired_robot_state: utils.robot_state):
        # calculate world relative robot velocity
        if self.past_obs is not None:
            world_relative_vel = self.local_to_world_vel(
                desired_robot_state.x_vel, desired_robot_state.y_vel
            )
        else:
            world_relative_vel = np.array([0, 0])

        action = {
            "infantry-0": np.array(
                [
                    world_relative_vel[0],
                    world_relative_vel[1],
                    desired_robot_state.angular_vel,
                    desired_robot_state.yaw,
                    desired_robot_state.pitch,
                ]
            ),
            "infantry-1": np.array([0, 0, 0, 0, 0]),
        }

        obs, reward, terminated, truncated, info = self.env.step(action=action)
        done = terminated or truncated
        self.env.render()

        self.past_obs = obs
        return obs

    def shutdown(self):
        self.env.close()

    def local_to_world_vel(self, x_vel, y_vel):
        yaw_rads = self.past_obs["agent"]["infantry-0"]["qpos"][0][3].item()
        rotation_matrix = np.array(
            [
                [np.cos(yaw_rads), -np.sin(yaw_rads)],
                [np.sin(yaw_rads), np.cos(yaw_rads)],
            ]
        )

        head_relative_vel = np.array([x_vel, y_vel])
        world_relative_vel = rotation_matrix @ head_relative_vel

        return world_relative_vel
