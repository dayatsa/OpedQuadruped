import __future__
import time
from ServoController import *

class Leg(ServoController):
    def __init__(self):
        ServoController.__init__(self)
        self.RAD_PER_DEG = 0.017453293
        self.MIN_DEGREE = -11.4592
        # self.MAX_DEGREE = 68.7549 #57.2958
        self.MAX_DEGREE = 97.4 #57.2958
        self.MOVE_STEP = 0.58
        self.MIDDLE_POSITION = 42.975

        self.lf = 0.0
        self.lh = 0.0
        self.rf = 0.0
        self.rh = 0.0
        self.leg_y = 0.0
        self.leg_x = 0.0
        self.last_leg_y = 0.0
        self.last_leg_x = 0.0
        self.last_lf = 0.0
        self.last_lh = 0.0
        self.last_rf = 0.0
        self.last_rh = 0.0
        self.leg_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint', 'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint', 'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint', 'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint']
        
        self.setInitialPositionMiddle()
        time.sleep(3)
        self.setMovingSpeed(500)
        # self.joint_group_publisher = rospy.Publisher('/oped/joint_group_position_controller/command', JointTrajectory, queue_size=1)
        # rospy.init_node('joint_states', anonymous=True)


    def degreeToRad(self, lf, lh, rf, rh):
        lf = lf * self.RAD_PER_DEG
        lh = lh * self.RAD_PER_DEG
        rf = rf * self.RAD_PER_DEG
        rh = rh * self.RAD_PER_DEG
        return lf, lh, rf, rh


    def addPosition(self, step_y, step_x):
        self.leg_y = self.leg_y + step_y*self.MOVE_STEP
        self.leg_x = self.leg_x + step_x*self.MOVE_STEP

        self.lf = self.MIDDLE_POSITION + self.leg_y - self.leg_x
        self.lh = self.MIDDLE_POSITION - self.leg_y - self.leg_x
        self.rf = self.MIDDLE_POSITION + self.leg_y + self.leg_x
        self.rh = self.MIDDLE_POSITION - self.leg_y + self.leg_x
        # print(self.lf, self.lh, self.rf, self.rh)

        if (self.lf > self.MAX_DEGREE or self.lf < self.MIN_DEGREE or self.lh > self.MAX_DEGREE or self.lh < self.MIN_DEGREE or self.rf > self.MAX_DEGREE or self.rf < self.MIN_DEGREE or self.rh > self.MAX_DEGREE or self.rh < self.MIN_DEGREE):
            self.lf = self.last_lf
            self.lh = self.last_lh
            self.rf = self.last_rf
            self.rh = self.last_rh
            # self.leg_y = self.last_leg_y
            # self.leg_x = self.last_leg_x

        if self.lf > self.MAX_DEGREE:
            self.lf = self.MAX_DEGREE
        if self.lh > self.MAX_DEGREE:
            self.lh = self.MAX_DEGREE
        if self.rf > self.MAX_DEGREE:
            self.rf = self.MAX_DEGREE
        if self.rh > self.MAX_DEGREE:
            self.rh = self.MAX_DEGREE

        if self.lf < self.MIN_DEGREE:
            self.lf = self.MIN_DEGREE
        if self.lh < self.MIN_DEGREE:
            self.lh = self.MIN_DEGREE
        if self.rf < self.MIN_DEGREE:
            self.rf = self.MIN_DEGREE
        if self.rh < self.MIN_DEGREE:
            self.rh = self.MIN_DEGREE          

        self.last_lf = self.lf
        self.last_lh = self.lh
        self.last_rf = self.rf
        self.last_rh = self.rh
        self.last_leg_y = self.leg_y
        self.last_leg_x = self.leg_x
        # print(self.lf, self.lh, self.rf, self.rh)
        self.setPosition(self.lf, self.lh, self.rf, self.rh, True)


    def setPosition(self, lf, lh, rf, rh, is_sync):
        lf_hip = 0
        lf_upper = lf
        lf_lower = -lf*3/2
        lh_hip = 0
        lh_upper = -lh
        lh_lower = lh*3/2
        rf_hip = 0
        rf_upper = rf
        rf_lower = -rf*3/2
        rh_hip = 0
        rh_upper = -rh
        rh_lower = rh*3/2
        data = [[lf_hip, lf_upper, lf_lower],
                [lh_hip, lh_upper, lh_lower],
                [rf_hip, rf_upper, rf_lower],
                [rh_hip, rh_upper, rh_lower]]

        self.setGoalPosition(data, is_sync)

    
    def setInitialPositionMiddle(self):
        self.leg_y = 0.0
        self.leg_x = 0.0
        self.lf = self.MIDDLE_POSITION
        self.lh = self.MIDDLE_POSITION
        self.rf = self.MIDDLE_POSITION
        self.rh = self.MIDDLE_POSITION
        self.setPosition(self.lf, self.lh, self.rf, self.rh, False)


    def getLegPosition(self):
        return self.lf, self.lh, self.rf, self.rh



