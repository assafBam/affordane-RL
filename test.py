#!/home/assaf/anaconda3/envs/air/bin/python3

from environment import AffordanceEnv
from stable_baselines3 import DDPG
env = AffordanceEnv()
model = DDPG.load("ddpg_affordance")
# model = DDPG.load("a2c_affordance")
for _ in range(20):
    obs, info = env.reset()
    action, _ = model.predict(obs)
    target = f"{info['target']}"
    print(f'target: {target: <10}\t action: {action}')

