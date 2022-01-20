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


class OpedTesting:
    def __init__(self):
        self.SAMPLE_BATCH_SIZE = 20
        self.EPISODES          = 5000

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

    
    def getFloorSetPoint(self, set_x, set_y):
        self.floor_position_x = 0
        self.floor_position_y = 0

        if set_x > 0:
            self.set_point_floor_x = set_x
            self.set_point_floor_x_adder = 1.0
            self.resudial_floor_x = self.set_point_floor_x % 1
        else:
            self.set_point_floor_x = set_x
            self.resudial_floor_x = -((-self.set_point_floor_x) % 1)
            self.set_point_floor_x_adder = -1.0

        if set_y > 0:
            self.set_point_floor_y = set_y
            self.set_point_floor_y_adder = 1.0
            self.resudial_floor_y = self.set_point_floor_y % 1
        else:
            self.set_point_floor_y = set_y
            self.resudial_floor_y = -((-self.set_point_floor_y) % 1)
            self.set_point_floor_y_adder = -1.0


    def resetEnvironment(self, set_x, set_y):
        # rospy.sleep(0.2)
        self.floor.setInitialPosition()
        self.oped.setInitialPosition()
        rospy.sleep(0.3)
        self.oped.resetWorld()
        rospy.sleep(0.5)
        state_y, state_x = self.oped.getStateY(), self.oped.getStateX()
        self.getFloorSetPoint(set_x, set_y)
        rospy.sleep(0.3)
        self.floorStep()
        return self.oped.getStateY(), self.oped.getStateX()


    def floorStep(self):        
        while (self.set_point_floor_x != 0):
            self.floor.setPosition(self.floor_position_y, self.floor_position_x)
            self.floor_position_x += self.set_point_floor_x_adder
            rospy.sleep(0.05)
            if self.set_point_floor_x > 0 and self.floor_position_x > self.set_point_floor_x or self.set_point_floor_x < 0 and self.floor_position_x < self.set_point_floor_x:
                self.floor_position_x -= self.set_point_floor_x_adder
                self.floor_position_x += self.resudial_floor_x
                self.floor.setPosition(self.floor_position_y, self.floor_position_x)
                rospy.sleep(0.05)
                print("floor x: {:.3f}, y: {:.3f}".format(self.floor_position_x, self.floor_position_y))
                break

        while (self.set_point_floor_y != 0):
            self.floor.setPosition(self.floor_position_y, self.floor_position_x)
            self.floor_position_y += self.set_point_floor_y_adder
            rospy.sleep(0.05)
            if self.set_point_floor_y > 0 and self.floor_position_y > self.set_point_floor_y or self.set_point_floor_y < 0 and self.floor_position_y < self.set_point_floor_y:
                self.floor_position_y -= self.set_point_floor_y_adder
                self.floor_position_y += self.resudial_floor_y
                self.floor.setPosition(self.floor_position_y, self.floor_position_x)
                rospy.sleep(0.05)
                print("floor x: {:.3f}, y: {:.3f}".format(self.floor_position_x, self.floor_position_y))
                break
        # print("floor x: {:.3f}, y: {:.3f}".format(self.floor_position_x, self.floor_position_y))    


    def saveRewardValue(self, my_dict):
        self.now = datetime.now()
        dt_string = self.now.strftime("%d-%m-%Y_%H-%M")

        path = "/home/dayatsa/data/skipsi/opedd_ws/src/OpedQuadruped/oped/oped_teleopp/rewards/test_train/reward_" + dt_string + ".json"
        with open(path, 'w') as fp:
            json.dump(my_dict, fp)


    def run(self):
        while(True):
            print("Reset Environment")
            state_y, state_x = self.resetEnvironment(0, 0)
            print("state: ", state_y, state_x)
            init_imu_x = state_x[1]
            init_imu_y = state_y[1]
            discrete_state_x = self.agent.getDiscreteState(state_x)
            discrete_state_y = self.agent.getDiscreteState(state_y)
            print("disecrete_state: ", discrete_state_y, discrete_state_x)
            val = raw_input("OK? (y/n) : ")
            if val == "y":
                break
        ep_rewards = []
        index_episode = 0
        set_floor_x = [i for i in range(-16,17,2)]
        set_floor_y = [i for i in range(-16,17,2)]
        try:
            # while(True):
            for set_x in set_floor_x:
                for set_y in set_floor_y:
                    # val = raw_input("Continue? (y/n) : ")
                    # if val == "n":
                    #     break
                    
                    print()     
                    print("setpoint: {}, {}".format(set_x, set_y))          
                    done = False
                    episode_reward_x = 0
                    episode_reward_y = 0
                    index = 0 
                    init_imu_x = 0
                    init_imu_y = 0
                    aggr_ep_rewards = {'index': [], 'imu_x': [], 'imu_y': [], 'servo_x': [], 'servo_y': [], 'act_x': [], 'act_y': []}
                
                    while(True):
                        print("Reset Environment")
                        state_y, state_x = self.resetEnvironment(set_x, set_y)
                        print("state: ", state_y, state_x)
                        init_imu_x = state_x[1]
                        init_imu_y = state_y[1]
                        discrete_state_x = self.agent.getDiscreteState(state_x)
                        discrete_state_y = self.agent.getDiscreteState(state_y)
                        print("disecrete_state: ", discrete_state_y, discrete_state_x)
                        # val = raw_input("OK? (y/n) : ")
                        # if val == "y":
                        break
                    
                    print("Starting..")
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

                        rate.sleep()    
                        discrete_state_x = new_discrete_state_x
                        discrete_state_y = new_discrete_state_y
                    
                    print("Episode {}, index: {}, # Reward-x:{}, # Reward-y:{}".format(index_episode, index, episode_reward_x, episode_reward_y))
                    # val = raw_input("is legs raised? (y/n) : ")
                    leg_raised = "n"
                    if ((next_state_x[0] > 54.2 or next_state_x[0] < -54.2) and (next_state_y[0] > 54.2 or next_state_y[0] < -54.2)):
                        leg_raised = "y"
                    dict_model = {"episode":index_episode,
                        "index":index,
                        "init_imu_x":init_imu_x,
                        "init_imu_y":init_imu_y,
                        "last_imu_x":next_state_x[1],
                        "last_imu_y":next_state_y[1],
                        "reward_x":episode_reward_x,
                        "reward_y":episode_reward_y,
                        "legs_raised":leg_raised,
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
