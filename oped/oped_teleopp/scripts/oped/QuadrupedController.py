#!/usr/bin/env python2
import __future__
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import time
import rospy
from Imu import *
from Leg import *

class QuadrupedController(Leg) : 
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.8
        self.MODEL_URDF = '/home/dayatsa/model_editor_models/oped/src/oped/oped_description/urdf/oped.urdf'
        self.ACTION_N = 3
        self.STATE_SPACE = 2
        self.MAX_EPISODE = 300
        self.LIMIT_UPRIGHT = 2.5
        self.IMU_MIN_DEGREE = -35
        self.IMU_MAX_DEGREE = 35
        self.episode_step = 0
        self.roll = 0
        self.pitch = 0
        print("masuk 1")
        Leg.__init__(self)
        imu_subscriber = rospy.Subscriber("/imu_oped/data", Imu, self.imuCallback)
        # Imu.__init__(self)


    def __str__(self):
        return str(self.x + ", " + self.y + ", " + self.z)


    def imuCallback(self, data):
        self.roll = data.orientation.x
        self.pitch = data.orientation.y


    def getImuData(self):
        return self.roll, self.pitch, 0


    def step(self, choice1, choice2):
        '''
        Gives us 3 total movement options.
        '''
        self.episode_step += 1

        step_y = 0
        step_x = 0

        if choice1 == 0:
            step_y = 0
        elif choice1 == 1:
            step_y = -1
        elif choice1 == 2:
            step_y = 1

        if choice2 == 0:
            step_x = 0
        elif choice2 == 1:
            step_x = 1
        elif choice2 == 2:
            step_x = -1

        self.addPosition(step_y, step_x)  
        time.sleep(0.023)

        # new_state_imu = self.getImuData()
        new_state_x = self.getStateX()
        new_state_y = self.getStateY()

        imu_x = new_state_x[1]
        imu_y = new_state_y[1]

        #reward
        reward_y = 0
        reward_x = 0

        if imu_x > -self.LIMIT_UPRIGHT and imu_x < self.LIMIT_UPRIGHT:
            reward_x = 10
        if imu_y > -self.LIMIT_UPRIGHT and imu_y < self.LIMIT_UPRIGHT:
            reward_y = 10
            
        done = False
        if (imu_x < self.IMU_MIN_DEGREE or imu_x > self.IMU_MAX_DEGREE):
            done = True
            rospy.loginfo("x imu")
        if (imu_y < self.IMU_MIN_DEGREE or imu_y > self.IMU_MAX_DEGREE):
            done = True
            rospy.loginfo("y imu")
        if self.episode_step >= self.MAX_EPISODE:
            done = True
            rospy.loginfo("max_episode")

        # rospy.loginfo("Step" + str(self.episode_step) + " : " + str(done))
        return new_state_y, new_state_x, reward_y, reward_x, done


    def getInfo(self):
        leg_position = self.getLegPosition()
        imu_data = self.getImuData()
        data = {'lf':leg_position[0],
                'lh':leg_position[1], 
                'rf':leg_position[2], 
                'rh':leg_position[3],
                'x':imu_data[0],
                'y':imu_data[1],
                'z':imu_data[2]}
        return data

    
    def getStateY(self):
        imu_data = self.getImuData()
        data = [self.leg_y, imu_data[1]]
        return data


    def getStateX(self):
        imu_data = self.getImuData()
        data = [self.leg_x, imu_data[0]]
        return data


    def resetEpisode(self):
        self.episode_step = 0