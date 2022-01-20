#!/usr/bin/env python2

from __future__ import print_function
import rospy
# import roslib; roslib.load_manifest('oped_teleopp')
import numpy as np
import matplotlib.pyplot as plt
import time
import random
import os
import numpy as np
import json
import time
from datetime import datetime
from Agent import *
from QuadrupedController import *


class OpedEngine:
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
        self.dt_start_string   = self.now.strftime("%d-%m-%Y_%H:%M")
        self.last_counter      = False
        self.counter_end       = 0
        self.max_avg_reward    = 12000
        self.lift              = False

    

    def resetEnvironment(self):
        # rospy.sleep(0.2)
        # self.oped.setInitialPosition()
        rospy.sleep(0.3)
        return self.oped.getStateY(), self.oped.getStateX()


    def saveRewardValue(self, my_dict):
        self.now = datetime.now()
        dt_string = self.now.strftime("%d-%m-%Y_%H-%M")

        dict_model   = {"lr":self.agent.LEARNING_RATE,
                        "gamma":self.agent.GAMMA,
                        "move_step":self.oped.MOVE_STEP,
                        "limit_upright":self.oped.LIMIT_UPRIGHT,
                        "action_size":self.oped.ACTION_N,
                        "start_date":self.dt_start_string,
                        "end_date":dt_string,
                        "rewards":my_dict}

        path = "/home/pi/oped_ws/src/OpedQuadruped/oped/oped_teleopp/rewards/y/reward_y_" + dt_string + ".json"
        with open(path, 'w') as fp:
            json.dump(dict_model, fp)


    def checkRobot(self, state_x, state_y):
        if (state_x[1] < -5 or state_x[1] > 5 or state_y[1] < -5 or state_y[1] > 5):
            if self.counter_end == 0:
                self.counter_end += 1 
                self.last_counter = True
            elif self.last_counter == True:
                self.counter_end += 1

        else:
            self.counter_end = 0
            self.last_counter = False


    def run(self):
        ep_rewards = []
        aggr_ep_rewards = {'ep': [], 'avg': [], 'max': [], 'min': []}
        try:
            for index_episode in range(self.EPISODES):
                print()
                # print("Reset Environment")
                state_y, state_x = self.resetEnvironment()
                # print("state: ", state_y, state_x)
                discrete_state_y = self.agent.getDiscreteState(state_y)
                discrete_state_x = self.agent.getDiscreteState(state_x)
                print("disecrete_state: ", discrete_state_y, discrete_state_x)

                self.checkRobot(state_x, state_y)

                # if self.counter_end <= 5:
                if True:
                    done = False
                    episode_reward = 0
                    index = 0 
                    # while not done:
                    while True:
                        """ pitch maju: -36.6 = 32.8 -> -34.3
                            pitch mundur: 41.3 = -37 -> 37.8

                            roll kiri: -54.3 = -47 -> -48
                            roll kanan: 43.8 = 40.3 -> 40.4

                        """
                        action_y = self.agent.action(discrete_state_y, is_y=True)
                        action_x = self.agent.action(discrete_state_x, is_y=False)
                        # action_y = 2
                        # action_x = 0

                        next_state_y, next_state_x, reward_y, reward_x, done = self.oped.step(action_y, action_x)
                        new_discrete_state_y = self.agent.getDiscreteState(next_state_y)
                        new_discrete_state_x = self.agent.getDiscreteState(next_state_x)
                        episode_reward = episode_reward + reward_x + reward_y

                        # print(self.oped.getImuData())
                        print("sx:[{:.2f}, {:.2f}], sy:[{:.2f}, {:.2f}], ax:{}, ay:{}, rx:{:.2f}, ry:{:.2f}".format(
                            next_state_x[0], next_state_x[1], next_state_y[0], next_state_y[1], action_x, action_y, reward_x, reward_y))
                        # index += 1
                        # if not done:
                            # self.agent.updateModel(discrete_state_y, new_discrete_state_y, action_y, reward_y, is_y=True)
                            # self.agent.updateModel(discrete_state_x, new_discrete_state_x, action_x, reward_x, is_y=False)
                        
                        # rate.sleep()    
                        discrete_state_y = new_discrete_state_y
                        discrete_state_x = new_discrete_state_x

                        done = False
                    
                    self.agent.updateExplorationRate(index_episode)
                    print("Episode {}, index: {}, # Reward: {}".format(index_episode, index, episode_reward))
                    print("Exploration: {}".format(self.agent.exploration_rate))
                
                    ep_rewards.append(episode_reward)
                    if not index_episode % self.STATS_EVERY:
                        average_reward = float(sum(ep_rewards[-self.STATS_EVERY:]))/self.STATS_EVERY
                        aggr_ep_rewards['ep'].append(index_episode)
                        aggr_ep_rewards['avg'].append(average_reward)
                        aggr_ep_rewards['max'].append(max(ep_rewards[-self.STATS_EVERY:]))
                        aggr_ep_rewards['min'].append(min(ep_rewards[-self.STATS_EVERY:]))
                        print("Episode: {}, average reward: {}, cur_max: {}".format(index_episode, average_reward, self.max_avg_reward))
                        ep_rewards = []
                        if(average_reward > self.max_avg_reward):
                            # self.agent.saveModel()
                            # self.saveRewardValue(aggr_ep_rewards)
                            self.max_avg_reward = average_reward

                else:
                    print("END TRAINING")
                    break

        finally:
            pass
            # self.agent.saveModel()
            # self.saveRewardValue(aggr_ep_rewards)

            # plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['avg'], label="average rewards")
            # plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['max'], label="max rewards")
            # plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['min'], label="min rewards")
            # plt.legend(loc=4)
            # plt.show()


if __name__ == "__main__":
    print(os.getcwd())
    rospy.init_node('engine', anonymous=True)
    rate = rospy.Rate(50) # 
    oped_agent = OpedEngine()
    oped_agent.run()

    oped = QuadrupedController()
    while True: 
        oped.addPosition(1,0)
        # print(oped.getImuData())
        rate.sleep()