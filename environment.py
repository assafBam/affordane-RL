import gymnasium as gym
import numpy as np
from gymnasium import spaces
from numpy import inf

class AffordanceEnv(gym.Env):
    metadata = {"render_modes":[], "render_fps":1}
    def __init__(self, *args, **kwargs):
        super().__init__()
        # other things
        self.action_space = spaces.Box(low=np.array([0,0]), high=np.array([100, 360]), dtype=np.float32) # can change to radians if needed
        self.observation_space = spaces.Box(low=np.array([-inf,-inf, 0, -inf, -inf, -inf,-inf]),high=np.array([inf, inf, 360, inf, inf, inf, inf]), dtype=np.float32)

    def step(self, action):
        pass #make action
        return observation, reward, terminated, truncated, info
    def reset(self, seed=None, options=None):
        pass
        return observation, info
    def render(self):
        pass
    def close(self):
        pass


if __name__ == '__main__':
    # test the environment
    from stable_baselines3.common.env_checker import check_env
    env = AffordanceEnv() # add parameters as needed
    check_env(env)
