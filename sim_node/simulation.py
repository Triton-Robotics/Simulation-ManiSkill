import gymnasium as gym
import mani_skill.envs
from sim_node import comp_field
import numpy as np


class Simulation:
    def __init__(self, seed=2930):
        self.env = gym.make(
            "comp_field",  # This should map to your registered environment
            render_mode="human",  # Use "human" to visualize, or "rgb_array" to render frames without GUI
            reward_mode="sparse",
            obs_mode="rgb+segmentation+position",
        )

        obs, _ = self.env.reset(seed=seed)  # Reset and get the initial observation
        keyframe = self.env.unwrapped.agent.keyframes["default"]
        self.env.unwrapped.agent.robot.set_pose(keyframe.pose)

    def step(self):
        action = self.env.action_space.sample()  # Take random actions
        action_test = np.array([0, 0, 0, 0, 0.0])
        obs, reward, terminated, truncated, info = self.env.step(None)
        done = terminated or truncated
        self.env.render()
        return obs

    def shutdown(self):
        self.env.close()
