#!/home/assaf/anaconda3/envs/air/bin/python3
import gymnasium as gym
import numpy as np
import sys
import os.path

from stable_baselines3 import DDPG, PPO, SAC
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback

from environment import AffordanceEnv

n_epoch = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
print(f'{n_epoch =}')
alg = sys.argv[2].lower() if len(sys.argv) > 2 else 'd'
alg_name = 'ddpg'

if alg.startswith('p'):
    alg_name='ppo'
elif alg.startswith('s'):
    alg_name='sac'

log_file_name = os.path.join('logs', f'log_{alg_name}_{n_epoch}.csv')
log = open(log_file_name, 'w')

env = AffordanceEnv(log_file=log)

# The noise objects for DDPG
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

if alg.startswith('p'):
    alg_name='ppo'
    model = PPO("MlpPolicy", env, verbose=1)
elif alg.startswith('s'):
    alg_name='sac'
    model = SAC("MlpPolicy", env, action_noise=action_noise, verbose=1)
else:
    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1)
checkpoint_callback = CheckpointCallback(save_freq=100, save_path='./checkpoints/',name_prefix=f'{alg_name}_{n_epoch}', save_vecnormalize=True)
model.learn(total_timesteps=n_epoch, log_interval=10, callback=checkpoint_callback)
model.save(f"{alg_name}_{n_epoch}e_final")
print('done')
env.print_rewards_graph()
