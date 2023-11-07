#!/home/assaf/anaconda3/envs/air/bin/python3
import gymnasium as gym
import numpy as np
import sys

from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback

from environment import AffordanceEnv, print_rewards_graph

env = AffordanceEnv()

n_epoch = int(sys.argv[1]) if len(sys.argv) > 1 else 1000

# The noise objects for DDPG
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
checkpoint_callback = CheckpointCallback(save_freq=100, save_path='./logs/',name_prefix='ddpg', save_vecnormalize=True)

model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1)
model.learn(total_timesteps=n_epoch, log_interval=10, callback=checkpoint_callback)
model.save("ddpg_affordance")
print('done')
print_rewards_graph()
