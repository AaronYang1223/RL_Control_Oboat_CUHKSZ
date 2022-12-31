from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import time

import gym
import multiprocessing as mp

env = make_vec_env("Hiboat-v0")

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=25000)
model.save("ppo_cartpole")

del model # remove to demonstrate saving and loading

model = PPO.load("ppo_cartpole")

obs = env.reset()

while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()