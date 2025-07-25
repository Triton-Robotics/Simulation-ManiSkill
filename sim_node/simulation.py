import gymnasium as gym
import mani_skill.envs
from mani_skill.envs.sapien_env import BaseEnv
from sim_node import comp_field
import numpy as np
from sim_node import utils


class Simulation:

    def __init__(self, seed=2930):
        self.env = gym.make(
            "comp_field",  # This should map to your registered environment
            render_mode="human",  # Use "human" to visualize, or "rgb_array" to render frames without GUI
            reward_mode="sparse",
            obs_mode="state_dict+rgb+segmentation+position",
            # obs_mode="rgb",
        )

        self.base_env: BaseEnv = self.env
        self.past_obs = None

        obs, _ = self.env.reset(seed=seed)  # Reset and get the initial observation
        keyframe = self.env.get_wrapper_attr("agent").keyframes["default"]
        self.env.get_wrapper_attr("agent").robot.set_pose(keyframe.pose)

    def step(self, desired_robot_state: utils.robot_state):
        # calculate world relative robot velocity
        if self.past_obs is not None:
            world_relative_vel = self.local_to_world_vel(
                desired_robot_state.x_vel, desired_robot_state.y_vel
            )
        else:
            world_relative_vel = np.array([0, 0])

        action = np.array(
            [
                # opposite indexes because I have been given robot cadd with y axis up and this is now my life
                world_relative_vel[1],
                world_relative_vel[0],
                0,
                desired_robot_state.yaw,
                desired_robot_state.pitch,
            ]
        )

        obs, reward, terminated, truncated, info = self.env.step(action)
        done = terminated or truncated
        self.env.render()

        self.past_obs = obs
        return obs

    def shutdown(self):
        self.env.close()

    def local_to_world_vel(self, x_vel, y_vel):
        yaw_rads = self.past_obs["agent"]["qpos"][0][3].item()

        rotation_matrix = np.array(
            [
                [np.cos(yaw_rads), -np.sin(yaw_rads)],
                [np.sin(yaw_rads), np.cos(yaw_rads)],
            ]
        )

        head_relative_vel = np.array([x_vel, y_vel])
        world_relative_vel = rotation_matrix @ head_relative_vel

        print(
            f"{yaw_rads:.3f},{x_vel:.3f},{y_vel:.3f},{world_relative_vel[0]:.3f},{world_relative_vel[1]:.3f}"
        )

        return world_relative_vel
