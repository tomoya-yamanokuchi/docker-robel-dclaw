import gym

import mujoco_py

env = gym.make("Hopper-v4")
o = env.reset()

for _ in range(1000):
    obs = env.render()
    a = env.action_space.sample()
    o, r, done, info = env.step(a)
    if done:
        o = env.reset()

env.close()