import gymnasium as gym
import mani_skill.envs
from sim_node import comp_field
from sim_node import infantry_robot

env = gym.make(
    "comp_field",  # This should map to your registered environment
    render_mode="human",  # Use "human" to visualize, or "rgb_array" to render frames without GUI
    reward_mode="sparse",
)

obs, _ = env.reset(seed=0)  # Reset and get the initial observation
keyframe = env.unwrapped.agent.keyframes["default"]
env.unwrapped.agent.robot.set_pose(keyframe.pose)
done = False


while not done:
    action = env.action_space.sample()  # Take random actions
    obs, reward, terminated, truncated, info = env.step(action)
    done = terminated or truncated
    env.render()

env.close()


# def main():
#   print("Hi from sim_node.")


# if __name__ == "__main__":
#    main()
