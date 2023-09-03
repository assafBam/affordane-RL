import gymnasium as gym
import numpy as np
import random
from gymnasium import spaces
from numpy import inf
from collections import namedtuple

from parameters import TIME_BOUND
from run import run_commands

TARGET_LOW_BOUND, TARGET_HIGH_BOUND = 0.5, 1.5 # in meters
# TARGETS = [(0,10),(0,0),(10,0),(2550,0),(2550,1),(2550,-1),(2551,0),(-2550,0),(-2550,1),(-2550,-1),(-2551,0)]
TARGETS = [(0,1.3),(1.3,0), (0,-1.3)]
# TARGETS = [(10,0),(0,0),(2550,0),(2551,0),(1000,0), (350,0)]
ROBOT_INIT_POS = (-1,-1, 0) # x,y,theta
BALL_INIT_POS = (0,0) # x,y
MAX_SPEED = 0.22 # meters per second
MASS_ROBOT = 1 # kg
MASS_BALL = 0.044 # kg
COMMANDS_FILE = "commands.txt"

Observation = namedtuple('Observation', ['robot_x', 'robot_y', 'robot_theta', 'ball_x', 'ball_y', 'target_x', 'target_y'])



def make_step(action: tuple[float, float])-> dict[str, float]:
    pass # returns the location of the ball and the location of the robot
    # MU = 0.4
    # g = 9.8
    # v_ball = np.sqrt(MASS_ROBOT/MASS_BALL)*action[0]
    # ell = v_ball * TIME_BOUND # x=vt
    # roll_theta = action[1]-np.pi # the ball rolling vector is 180 degrees diferrent from the angle of which the robot impact it
    # x,y = ell*np.cos(roll_theta), ell*np.sin(roll_theta)
    # return {'robot_x':BALL_INIT_POS[0],'robot_y':BALL_INIT_POS[1],'robot_theta':action[1],'ball_x':x,'ball_y':y}
    ball_pos = run_commands(action[0], action[1], COMMANDS_FILE)
    return {'robot_x':BALL_INIT_POS[0],'robot_y':BALL_INIT_POS[1],'robot_theta':action[1],'ball_x':ball_pos[0],'ball_y':ball_pos[1]}

def distance(p1, p2):
    return np.sqrt(np.power(p1[0]-p2[0],2)+np.power(p1[1]-p2[1], 2))

def calculate_reward(ball_loc, target): #TODO: save all rewards and print them to a nice graph
    # EPSILON = 10
    # tmp = np.sqrt(np.power(ball_loc[0]-target[0],2)+np.power(ball_loc[1]-target[1], 2))
    # rewards.append(EPSILON - tmp**3)
    # return EPSILON - tmp**3
    GAMMA = 10
    res = -np.log(GAMMA*distance(ball_loc, target))
    return res


class AffordanceEnv(gym.Env):
    metadata = {"render_modes":[], "render_fps":1}
    def __init__(self, *args, log_file=None, **kwargs):
        super().__init__()
        # other things
        #   action: V,theta
        #TODO: change to lower speed boundries?
        self.action_space = spaces.Box(low=np.array([0,0]), high=np.array([MAX_SPEED, 2* np.pi]), dtype=np.float32) # angle in radians
        # self.action_space = spaces.Box(low=np.array([0,0]), high=np.array([100, 0.1]), dtype=np.float32)
        #   observation: robot_x, robot_y, robot_theta, ball_x, ball_y, target_x, target_y
        self.observation_space = spaces.Box(low=np.array([-inf,-inf, 0, -inf, -inf, -inf,-inf]),high=np.array([inf, inf, 360, inf, inf, inf, inf]), dtype=np.float32)
        self.target = None
        self.rewards=[]
        self.log_file=log_file
        if self.log_file:
            print('target,action,reward', file=self.log_file)

        self.index = 0

    def step(self, action):
        locations = make_step(action)
        obs = Observation(robot_x=locations['robot_x'],robot_y=locations['robot_y'], robot_theta=locations['robot_theta'], ball_x=locations['ball_x'], ball_y=locations['ball_y'], target_x=self.target[0], target_y=self.target[1])
        reward = calculate_reward(ball_loc=(locations['ball_x'],locations['ball_y']), target=self.target)
        terminated = True
        truncated = False
        info = {'target':self.target, 'action':action}
        self.rewards.append(reward)
        if self.log_file:
            print(f'{self.index},{str(self.target).replace(",",";")},{action},{reward}', file=self.log_file)
        return np.array(obs, dtype=np.float32), reward, terminated, truncated, info
    def reset(self, seed=None, options=None):
        pass
        self.target = random.choice(TARGETS)
        obs = Observation(robot_x=ROBOT_INIT_POS[0],robot_y=ROBOT_INIT_POS[1], robot_theta=ROBOT_INIT_POS[2], ball_x=BALL_INIT_POS[0], ball_y=BALL_INIT_POS[1], target_x=self.target[0], target_y=self.target[1])
        info = {'target':self.target}
        return np.array(obs, dtype=np.float32), info
    def render(self):
        pass
    def close(self):
        pass
    def print_rewards_graph(self):
        import matplotlib.pyplot as plt
        y=np.array(self.rewards)
        plt.plot(y)
        plt.ylabel('reward')
        plt.xlabel('epoch')
        plt.title('reward as a function of the epoch')
        plt.show()



if __name__ == '__main__':
    # test the environment
    from stable_baselines3.common.env_checker import check_env
    env = AffordanceEnv() # add parameters as needed
    check_env(env)
