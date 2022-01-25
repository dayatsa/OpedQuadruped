#!/usr/bin/env python2
import __future__
import time
from Ax12 import Ax12
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ServoController(object):
    def __init__(self):
        """class for Oped servo controller using Dynamixel AX-12"""
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
        Ax12.DEVICENAME = '/dev/ttyUSB0'
        Ax12.BAUDRATE = 1000000
        # sets baudrate and opens com port
        Ax12.connect()

        self.FEMUR_ADDER = 21.772396214
        self.TIBIA_ADDER = 28.647889755

        self.lf_coxa = Ax12(3)
        self.lf_femur = Ax12(4)
        self.lf_tibia = Ax12(5)

        self.lh_coxa = Ax12(9)
        self.lh_femur = Ax12(10)
        self.lh_tibia = Ax12(11)

        self.rf_coxa = Ax12(0)
        self.rf_femur = Ax12(1)
        self.rf_tibia = Ax12(2)

        self.rh_coxa = Ax12(6)
        self.rh_femur = Ax12(7)
        self.rh_tibia = Ax12(8)
        
        self.servo = Ax12([3,4,5,9,10,11,0,1,2,6,7,8])

        self.setMovingSpeed(100)

        self.init_position =[[150.0, 150.0, 150.0], #lf
                            [150.0, 150.0, 150.0], #lh
                            [150.0, 150.0, 150.0], #rf
                            [150.0, 150.0, 150.0]] #rh

        self.position_adder =  [[0, -68.2276, 58.4417], #lf
                                [0, 68.2276, -58.4417], #lh
                                [0, 68.2276, -58.4417], #rf
                                [0, -68.2276, 58.4417]] #rh

        self.goal_position_raw =   [[0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]]

        self.goal_position_deg =   [[0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]]
        
        self.raw_positions = []

        
    

    def servoCallback(self, data):
        rospy.loginfo("callback")
        # rospy.loginfo(data)
        self.raw_positions = data.points[0].positions
        data = [[self.raw_positions[0], self.raw_positions[1], self.raw_positions[2]],
                [self.raw_positions[3], self.raw_positions[4], self.raw_positions[5]],
                [self.raw_positions[6], self.raw_positions[7], self.raw_positions[8]],
                [self.raw_positions[9], self.raw_positions[10], self.raw_positions[11]]]
        self.setGoalPosition(data, True)


    def setMovingSpeed(self, speed):
        """set moving speed dynamixel"""
        self.lf_coxa.set_moving_speed(speed)
        self.lf_femur.set_moving_speed(speed)
        self.lf_tibia.set_moving_speed(speed)
        self.lh_coxa.set_moving_speed(speed)
        self.lh_femur.set_moving_speed(speed)
        self.lh_tibia.set_moving_speed(speed)
        self.rf_coxa.set_moving_speed(speed)
        self.rf_femur.set_moving_speed(speed)
        self.rf_tibia.set_moving_speed(speed)
        self.rh_coxa.set_moving_speed(speed)
        self.rh_femur.set_moving_speed(speed)
        self.rh_tibia.set_moving_speed(speed)
        # self.servo.set_moving_speed(speed)


    def deg2raw(self):
        """convert position value from degree to raw(0-1025)"""
        for i in range(4):
            for j in range(3):
                self.goal_position_raw[i][j] = self.servo.deg2raw(self.goal_position_deg[i][j])
        

    def setPointCalibration(self, data):
        """adding calibration to position servo refered to actual position"""
        self.goal_position_deg[0][0] = self.init_position[0][0] + self.position_adder[0][0] + data[0][0]
        self.goal_position_deg[0][1] = self.init_position[0][1] + self.position_adder[0][1] + data[0][1]
        self.goal_position_deg[0][2] = self.init_position[0][2] + self.position_adder[0][2] + data[0][2]

        self.goal_position_deg[1][0] = self.init_position[1][0] + self.position_adder[1][0] + data[1][0]
        self.goal_position_deg[1][1] = self.init_position[1][1] + self.position_adder[1][1] + data[1][1]
        self.goal_position_deg[1][2] = self.init_position[1][2] + self.position_adder[1][2] + data[1][2]

        self.goal_position_deg[2][0] = self.init_position[2][0] + self.position_adder[2][0] - data[2][0]
        self.goal_position_deg[2][1] = self.init_position[2][1] + self.position_adder[2][1] - data[2][1]
        self.goal_position_deg[2][2] = self.init_position[2][2] + self.position_adder[2][2] - data[2][2]

        self.goal_position_deg[3][0] = self.init_position[3][0] + self.position_adder[3][0] - data[3][0]
        self.goal_position_deg[3][1] = self.init_position[3][1] + self.position_adder[3][1] - data[3][1]
        self.goal_position_deg[3][2] = self.init_position[3][2] + self.position_adder[3][2] - data[3][2]

        self.deg2raw()


    def setGoalPosition(self, data, is_sync):
        """set goal position\n
           input list with row x column: 4x3
        """
        self.setPointCalibration(data)

        if is_sync:
            data_send = []
            for i in range(4):
                for j in range(3):
                    data_send.append(self.goal_position_raw[i][j])
            self.servo.syncWriteJoints(data_send)
        else:
            self.sendManual(self.goal_position_raw)


    def sendManual(self, data):
        self.lf_coxa.set_goal_position(data[0][0])
        self.lf_femur.set_goal_position(data[0][1])
        self.lf_tibia.set_goal_position(data[0][2])
        self.lh_coxa.set_goal_position(data[1][0])
        self.lh_femur.set_goal_position(data[1][1])
        self.lh_tibia.set_goal_position(data[1][2])
        self.rf_coxa.set_goal_position(data[2][0])
        self.rf_femur.set_goal_position(data[2][1])
        self.rf_tibia.set_goal_position(data[2][2])
        self.rh_coxa.set_goal_position(data[3][0])
        self.rh_femur.set_goal_position(data[3][1])
        self.rh_tibia.set_goal_position(data[3][2])
        time.sleep(3)


    def setInitialPosition(self):        
        data = [[0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]]

        #femur -0.38 = 21.772396214 derajat 
        #tibia +0.5 = 28.647889755 derajat
        self.setGoalPosition(data)


if __name__ == '__main__':
    try:
        print("masuk servo controller")
        # time.sleep(3)
        rospy.init_node('servo_controller', anonymous=True)
        sevo = ServoController()
        joint_subsriber = rospy.Subscriber("/oped/joint_group_position_controller/command", JointTrajectory, sevo.servoCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass