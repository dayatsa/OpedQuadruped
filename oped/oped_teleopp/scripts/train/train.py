#!/usr/bin/env python2

from __future__ import print_function
import rospy
import roslib; roslib.load_manifest('oped_teleopp')
import numpy as np
import matplotlib.pyplot as plt
import time
import random
import os
import numpy as np
import json
from datetime import datetime
from agent import *
from quadruped_controller import *
from floor_controller import *


class OpedTrainer:
    def __init__(self):
        self.SAMPLE_BATCH_SIZE = 20
        self.EPISODES          = 10000

        self.oped              = Quadruped()
        self.floor             = Floor()
        self.STATE_SPACE       = self.oped.STATE_SPACE
        self.ACTION_SIZE       = self.oped.ACTION_N
        self.MAX_EPISODE       = self.oped.MAX_EPISODE
        self.STATS_EVERY       = 20
        self.agent             = Agent(self.STATE_SPACE, self.ACTION_SIZE, self.EPISODES)        
        self.floor_position_x  = 0
        self.floor_position_y  = 0
        self.set_point_floor_x  = 0
        self.set_point_floor_y  = 0
        self.resudial_floor_x  = 0
        self.resudial_floor_y  = 0
        self.set_point_floor_x_adder = 0
        self.set_point_floor_y_adder = 0
        self.now               = datetime.now()
        self.dt_start_string   = self.now.strftime("%d-%m-%Y_%H:%M")
        self.last_counter      = False
        self.counter_end       = 0
        self.max_avg_reward    = 1000
        self.lift              = False

    
    def getFloorSetPoint(self):
        self.floor_position_x = 0
        self.floor_position_y = 0


        # if np.random.rand() <= 0.5:
        if self.lift == True:
            self.set_point_floor_x = np.random.uniform(1.0, 20.0)
            self.set_point_floor_x_adder = 1.0
            self.resudial_floor_x = self.set_point_floor_x % 1
            self.lift = False
        else:
            self.set_point_floor_x = -np.random.uniform(1.0, 20.0)
            self.resudial_floor_x = -((-self.set_point_floor_x) % 1)
            self.set_point_floor_x_adder = -1.0
            self.lift = True

        # if np.random.rand() <= 0.5:
        # if self.lift == True:
        #     self.set_point_floor_y = np.random.uniform(5.0, 20.0)
        #     self.set_point_floor_y_adder = 1.0
        #     self.resudial_floor_y = self.set_point_floor_y % 1
        #     self.lift = False
        # else:
        #     self.set_point_floor_y = -np.random.uniform(5.0, 20.0)
        #     self.resudial_floor_y = -((-self.set_point_floor_y) % 1)
        #     self.set_point_floor_y_adder = -1.0
        #     self.lift = True
        
        # print("set_point_floor x: {:.3f}, y: {:.3f}".format(self.set_point_floor_x, self.set_point_floor_y))


    def resetEnvironment(self):
        # rospy.sleep(0.2)
        self.floor.setInitialPosition()
        self.oped.setInitialPosition()
        rospy.sleep(0.3)
        self.oped.resetWorld()
        rospy.sleep(0.5)
        state_y, state_x = self.oped.getStateY(), self.oped.getStateX()
        self.getFloorSetPoint()
        rospy.sleep(0.3)
        self.floorStep()
        return state_y, state_x

    
    def floorStep(self):        
        while (self.set_point_floor_x != 0):
            self.floor.setPosition(self.floor_position_y, self.floor_position_x)
            self.floor_position_x += self.set_point_floor_x_adder
            rospy.sleep(0.05)
            if self.set_point_floor_x > 0 and self.floor_position_x >= self.set_point_floor_x or self.set_point_floor_x < 0 and self.floor_position_x <= self.set_point_floor_x:
                self.floor_position_x -= self.set_point_floor_x_adder
                self.floor_position_x += self.resudial_floor_x
                self.floor.setPosition(self.floor_position_y, self.floor_position_x)
                rospy.sleep(0.05)
                # print("floor x: {:.3f}, y: {:.3f}".format(self.floor_position_x, self.floor_position_y))
                break

        while (self.set_point_floor_y != 0):
            self.floor.setPosition(self.floor_position_y, self.floor_position_x)
            self.floor_position_y += self.set_point_floor_y_adder
            rospy.sleep(0.05)
            if self.set_point_floor_y > 0 and self.floor_position_y >= self.set_point_floor_y or self.set_point_floor_y < 0 and self.floor_position_y <= self.set_point_floor_y:
                self.floor_position_y -= self.set_point_floor_y_adder
                self.floor_position_y += self.resudial_floor_y
                self.floor.setPosition(self.floor_position_y, self.floor_position_x)
                rospy.sleep(0.05)
                # print("floor x: {:.3f}, y: {:.3f}".format(self.floor_position_x, self.floor_position_y))
                break
    

    def saveRewardValue(self, my_dict):
        self.now = datetime.now()
        dt_string = self.now.strftime("%d-%m-%Y_%H:%M")

        dict_model   = {"lr":self.agent.LEARNING_RATE,
                        "gamma":self.agent.GAMMA,
                        "move_step":self.oped.MOVE_STEP,
                        "limit_upright":self.oped.LIMIT_UPRIGHT,
                        "action_size":self.oped.ACTION_N,
                        "start_date":self.dt_start_string,
                        "end_date":dt_string,
                        "rewards":my_dict}

        path = "/home/dayatsa/data/skipsi/opedd_ws/src/OpedQuadruped/oped/oped_teleopp/rewards/x/reward_x_" + dt_string + ".json"
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
                print("init state: {}, {}".format(state_x, state_y))
                print("start state: {}, {}".format(self.oped.getStateX(), self.oped.getStateY()))
                discrete_state_y = self.agent.getDiscreteState(state_y)
                discrete_state_x = self.agent.getDiscreteState(state_x)
                # print("disecrete_state: ", discrete_state_y, discrete_state_x)

                self.checkRobot(state_x, state_y)

                if self.counter_end <= 5:
                    done = False
                    episode_reward = 0
                    index = 0 
                    while not done:
                    # while True:
                        """ action y:
                            0: imu 0
                            1: imu minus -26.77
                            2: imu plus 26.77
                            action x:
                            0: imu 0
                            1: imu minus -28.9348
                        """
                        # action_y = self.agent.action(discrete_state_y, is_y=True)
                        action_x = self.agent.action(discrete_state_x, is_y=False)
                        action_y = 0
                        # action_x = 0

                        next_state_y, next_state_x, reward_y, reward_x, done = self.oped.step(action_y, action_x)
                        new_discrete_state_y = self.agent.getDiscreteState(next_state_y)
                        new_discrete_state_x = self.agent.getDiscreteState(next_state_x)
                        episode_reward = episode_reward + reward_x #+ reward_y

                        # if index < 450 :
                        # self.floorStep()
                        # print(next_state_y, action_y, reward_y)
                        # print("sx:[{:.2f}, {:.2f}], sy:[{:.2f}, {:.2f}], ax:{}, ay:{}, rx:{:.2f}, ry:{:.2f}".format(
                        #     next_state_x[0], next_state_x[1], next_state_y[0], next_state_y[1], action_x, action_y, reward_x, reward_y))
                        # print(self.oped.getInfo())
                        # print(self.floor_position_y)
                        index += 1
                        if not done:
                            # self.agent.updateModel(discrete_state_y, new_discrete_state_y, action_y, reward_y, is_y=True)
                            self.agent.updateModel(discrete_state_x, new_discrete_state_x, action_x, reward_x, is_y=False)
                        
                        rate.sleep()    
                        discrete_state_y = new_discrete_state_y
                        discrete_state_x = new_discrete_state_x
                    
                    self.agent.updateExplorationRate(index_episode)
                    print("finish state: {}, {}".format(self.oped.getStateX(), self.oped.getStateY()))
                    print("Episode {}, index: {}, # Reward: {}".format(index_episode, index, episode_reward))
                    print("Exploration: {}, x: {:.3f}, y: {:.3f}".format(self.agent.exploration_rate, self.floor_position_x, self.floor_position_y))
                
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
                            self.agent.saveModel()
                            self.saveRewardValue(aggr_ep_rewards)
                            self.max_avg_reward = average_reward

                else:
                    print("END TRAINING")
                    break

        finally:
            self.agent.saveModel()
            self.saveRewardValue(aggr_ep_rewards)

            plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['avg'], label="average rewards")
            plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['max'], label="max rewards")
            plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['min'], label="min rewards")
            plt.legend(loc=4)
            plt.show()


if __name__ == "__main__":
    rospy.init_node('train', anonymous=True)
    rate = rospy.Rate(50) # 
    oped_agent = OpedTrainer()
    oped_agent.run()