#!/usr/bin/env python2

from __future__ import print_function
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import random
import os
import numpy as np
import json
from datetime import datetime
from Agent import *
from QuadrupedController import *


class OpedTesting:
    def __init__(self):
        self.SAMPLE_BATCH_SIZE = 20
        self.EPISODES          = 5000

        self.oped              = QuadrupedController()
        self.STATE_SPACE       = self.oped.STATE_SPACE
        self.ACTION_SIZE       = self.oped.ACTION_N
        self.MAX_EPISODE       = self.oped.MAX_EPISODE
        self.STATS_EVERY       = 20
        self.agent             = Agent(self.STATE_SPACE, self.ACTION_SIZE, self.EPISODES)        
        self.now               = datetime.now()
        self.dt_start_string   = self.now.strftime("%d-%m-%Y_%H-%M")

    

    def resetEnvironment(self):
        # rospy.sleep(0.2)
        self.oped.resetEpisode()
        self.oped.setInitialPositionMiddle()
        rospy.sleep(0.3)
        return self.oped.getStateY(), self.oped.getStateX()


    def saveRewardValue(self, my_dict):
        self.now = datetime.now()
        dt_string = self.now.strftime("%d-%m-%Y_%H-%M")

        path = "/home/pi/oped_ws/src/OpedQuadruped/oped/oped_teleopp/rewards/test_oped/reward_" + dt_string + ".json"
        with open(path, 'w') as fp:
            json.dump(my_dict, fp)


    def run(self):
        ep_rewards = []
        index_episode = 0
        try:
            while(True):
                val = raw_input("Continue? (y/n) : ")
                if val == "n":
                    break
                
                print()
                done = False
                episode_reward_x = 0
                episode_reward_y = 0
                index = 0 
                init_imu_x = 0
                init_imu_y = 0
                aggr_ep_rewards = {'index': [], 'imu_x': [], 'imu_y': [], 'servo_x': [], 'servo_y': [], 'act_x': [], 'act_y': []}
               
                while(True):
                    print("Reset Environment")
                    for i in range(50):
                        self.oped.getImuData()
                    state_y, state_x = self.resetEnvironment()
                    print("state: ", state_y, state_x)
                    print(self.oped.getImuData())
                    discrete_state_x = self.agent.getDiscreteState(state_x)
                    discrete_state_y = self.agent.getDiscreteState(state_y)
                    print("disecrete_state: ", discrete_state_y, discrete_state_x)
                    val = raw_input("OK? (y/n)")
                    if val == "y":
                        break
                
                print("Starting..")

                aggr_ep_rewards = {'index': [], 'imu_x': [], 'imu_y': [], 'servo_x': [], 'servo_y': [], 'act_x': [], 'act_y': []}
                while not done:
                    action_x = self.agent.action(discrete_state_x, is_y=False)
                    action_y = self.agent.action(discrete_state_y, is_y=True)
                    next_state_y, next_state_x, reward_y, reward_x, done = self.oped.step(action_y, action_x)
                    new_discrete_state_x = self.agent.getDiscreteState(next_state_x)
                    new_discrete_state_y = self.agent.getDiscreteState(next_state_y)
                    episode_reward_x = episode_reward_x + reward_x
                    episode_reward_y = episode_reward_y + reward_y
                    
                    print("sx:[{:.2f}, {:.2f}], sy:[{:.2f}, {:.2f}], ax:{}, ay:{}, rx:{:.2f}, ry:{:.2f}".format(
                        next_state_x[0], next_state_x[1], next_state_y[0], next_state_y[1], action_x, action_y, reward_x, reward_y))
                    aggr_ep_rewards['index'].append(index)
                    aggr_ep_rewards['imu_x'].append(next_state_x[1])
                    aggr_ep_rewards['imu_y'].append(next_state_y[1])
                    aggr_ep_rewards['servo_x'].append(next_state_x[0])
                    aggr_ep_rewards['servo_y'].append(next_state_y[0])
                    aggr_ep_rewards['act_x'].append(action_x)
                    aggr_ep_rewards['act_y'].append(action_y)
                    index += 1

                    # rate.sleep()    
                    discrete_state_x = new_discrete_state_x
                    discrete_state_y = new_discrete_state_y
                
                print("Episode {}, index: {}, # Reward-x:{}, # Reward-y:{}".format(index_episode, index, episode_reward_x, episode_reward_y))
                val = raw_input("is legs raised? (y/n) : ")
                dict_model = {"episode":index_episode,
                    "index":index,
                    "init_imu_x":init_imu_x,
                    "init_imu_y":init_imu_y,
                    "last_imu_x":next_state_x[1],
                    "last_imu_y":next_state_y[1],
                    "reward_x":episode_reward_x,
                    "reward_y":episode_reward_y,
                    "legs raised":val,
                    "data":aggr_ep_rewards}


                ep_rewards.append(dict_model)
                index_episode+=1

        finally:
            self.saveRewardValue(ep_rewards)


if __name__ == "__main__":
    print(os.getcwd())
    rospy.init_node('engine', anonymous=True)
    rate = rospy.Rate(50) # 
    oped_agent = OpedTesting()
    oped_agent.run()
