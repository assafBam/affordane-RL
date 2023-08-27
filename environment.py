import gymnasium as gym
import numpy as np
from gymnasium import spaces
from numpy import inf
from collections import namedtuple

TARGET_LOW_BOUND, TARGET_HIGH_BOUND = 0.01, 0.2 # in meters
ROBOT_INIT_POS = (-1,-1, 0) # x,y,theta
BALL_INIT_POS = (0,0) # x,y

Observation = namedtuple('Observation', ['robot_x', 'robot_y', 'robot_theta', 'ball_x', 'ball_y', 'target_x', 'target_y'])

def make_step(action)-> dict[str, float]:
    pass # returns the location of the ball and the location of the robot
    MU = 0.2
    g = 9.8
    ell = np.power(action[0], 2)/(2*MU*g)
    x,y = ell*np.cos(np.deg2rad(action[1])), ell*np.sin(np.deg2rad(action[1]))
    return {'robot_x':BALL_INIT_POS[0],'robot_y':BALL_INIT_POS[1],'robot_theta':action[1],'ball_x':x,'ball_y':y}
def calculate_reward(ball_loc, target):
    EPSILON = 0.1
    return EPSILON - np.sqrt(np.power(ball_loc[0]-target[0],2)+np.power(ball_loc[1]-target[1], 2))

class AffordanceEnv(gym.Env):
    metadata = {"render_modes":[], "render_fps":1}
    def __init__(self, *args, **kwargs):
        super().__init__()
        # other things
        #   action: V,theta
        self.action_space = spaces.Box(low=np.array([0,0]), high=np.array([100, 360]), dtype=np.float32) # can change to radians if needed
        #   observation: robot_x, robot_y, robot_theta, ball_x, ball_y, target_x, target_y
        self.observation_space = spaces.Box(low=np.array([-inf,-inf, 0, -inf, -inf, -inf,-inf]),high=np.array([inf, inf, 360, inf, inf, inf, inf]), dtype=np.float32)
        
        self.target = None

    def step(self, action):
        locations = make_step(action)
        obs = Observation(robot_x=locations['robot_x'],robot_y=locations['robot_y'], robot_theta=locations['robot_theta'], ball_x=locations['ball_x'], ball_y=locations['ball_y'], target_x=self.target[0], target_y=self.target[1])
        reward = calculate_reward(ball_loc=(locations['ball_x'],locations['ball_y']), target=self.target)
        terminated = True
        truncated = False
        info = {'target':self.target, 'action':action}
        return np.array(obs, dtype=np.float32), reward, terminated, truncated, info
    def reset(self, seed=None, options=None):
        pass
        self.target = np.random.random(size=2)*(TARGET_HIGH_BOUND-TARGET_LOW_BOUND)+TARGET_LOW_BOUND
        obs = Observation(robot_x=ROBOT_INIT_POS[0],robot_y=ROBOT_INIT_POS[1], robot_theta=ROBOT_INIT_POS[2], ball_x=BALL_INIT_POS[0], ball_y=BALL_INIT_POS[1], target_x=self.target[0], target_y=self.target[1])
        info = {'target':self.target}
        return np.array(obs, dtype=np.float32), info
    def render(self):
        pass
    def close(self):
        pass


if __name__ == '__main__':
    # test the environment
    from stable_baselines3.common.env_checker import check_env
    env = AffordanceEnv() # add parameters as needed
    check_env(env)
