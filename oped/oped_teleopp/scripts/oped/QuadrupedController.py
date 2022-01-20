import __future__
import rospy
from Imu import *
from Leg import *

class QuadrupedController(Leg, Imu) : 
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.8
        self.MODEL_URDF = '/home/dayatsa/model_editor_models/oped/src/oped/oped_description/urdf/oped.urdf'
        self.ACTION_N = 3
        self.STATE_SPACE = 2
        self.MAX_EPISODE = 300
        self.episode_step = 0
        Leg.__init__(self)
        Imu.__init__(self)


    def __str__(self):
        return str(self.x + ", " + self.y + ", " + self.z)


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

        new_state_imu = self.getImuData()
        y = new_state_imu[1]
        x = new_state_imu[0]

        #reward
        reward_y = 0
        reward_x = 0

        if y > -self.LIMIT_UPRIGHT and y < self.LIMIT_UPRIGHT:
            reward_y += 10
        # else:
        #     if y < 0:
        #         reward_y += y
        #     else:
        #         reward_y -= y
        
        if x > -self.LIMIT_UPRIGHT and x < self.LIMIT_UPRIGHT:
            reward_x += 10
        # else:
        #     if x < 0:
        #         reward_x += x
        #     else:
        #         reward_x -= x

        done = False
        if (x < self.IMU_MIN_DEGREE or x > self.IMU_MAX_DEGREE):
            done = True
            rospy.loginfo("x imu")
        if (y < self.IMU_MIN_DEGREE or y > self.IMU_MAX_DEGREE):
            done = True
            rospy.loginfo("y imu")
        if self.episode_step >= self.MAX_EPISODE:
            done = True
            rospy.loginfo("max_episode")

        # rospy.loginfo("Step" + str(self.episode_step) + " : " + str(done))
        return self.getStateY(), self.getStateX(), reward_y, reward_x, done


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