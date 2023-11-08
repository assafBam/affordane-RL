#!/home/assaf/anaconda3/envs/air/bin/python3

from environment import AffordanceEnv
from stable_baselines3 import DDPG, PPO, SAC
import sys
import os.path

env = AffordanceEnv()

n_epoch = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
alg = sys.argv[2].lower() if len(sys.argv) > 2 else 'd'
alg_name = 'ddpg'

ALG=DDPG
if alg.startswith('p'):
    ALG = PPO
    alg_name='ppo'
elif alg.startswith('s'):
    ALG = SAC
    alg_name='sac'

checkpoint = os.path.join('checkpoints', f"{alg_name}_{n_epoch}e_final")
model = ALG.load(checkpoint)
for _ in range(20):
    obs, info = env.reset()
    action, _ = model.predict(obs)
    target = f"{info['target']}"
    print(f'target: {target: <10}\t action: {action}')

