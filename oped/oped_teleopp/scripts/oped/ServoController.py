from Ax12 import Ax12


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

        self.lf_coxa = Ax12(0)
        self.lf_femur = Ax12(1)
        self.lf_tibia = Ax12(2)

        self.lh_coxa = Ax12(3)
        self.lh_femur = Ax12(4)
        self.lh_tibia = Ax12(5)

        self.rf_coxa = Ax12(6)
        self.rf_femur = Ax12(7)
        self.rf_tibia = Ax12(8)

        self.rh_coxa = Ax12(9)
        self.rh_femur = Ax12(10)
        self.rh_tibia = Ax12(11)

        self.setMovingSpeed(500)

        self.position =[[150.0, 150.0, 150.0], #lf
                        [150.0, 150.0, 150.0], #lh
                        [150.0, 150.0, 150.0], #rf
                        [150.0, 150.0, 150.0]] #rh

        self.position_adder =  [[0, 0, 0], #lf
                                [0, 0, 0], #lh
                                [0, 0, 0], #rf
                                [0, 0, 0]] #rh

        self.goal_position_raw =   [[0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]]

        self.goal_position_deg =   [[0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]]


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


    def deg2raw(self):
        """convert position value from degree to raw(0-1025)"""
        self.goal_position_raw[0][0] = self.lf_coxa.deg2raw(self.goal_position_deg[0][0])
        self.goal_position_raw[0][1] = self.lf_femur.deg2raw(self.goal_position_deg[0][1])
        self.goal_position_raw[0][2] = self.lf_tibia.deg2raw(self.goal_position_deg[0][2])

        self.goal_position_raw[1][0] = self.lh_coxa.deg2raw(self.goal_position_deg[1][0])
        self.goal_position_raw[1][1] = self.lh_femur.deg2raw(self.goal_position_deg[1][1])
        self.goal_position_raw[1][2] = self.lh_tibia.deg2raw(self.goal_position_deg[1][2])

        self.goal_position_raw[2][0] = self.rf_coxa.deg2raw(self.goal_position_deg[2][0])
        self.goal_position_raw[2][1] = self.rf_femur.deg2raw(self.goal_position_deg[2][1])
        self.goal_position_raw[2][2] = self.rf_tibia.deg2raw(self.goal_position_deg[2][2])

        self.goal_position_raw[3][0] = self.rh_coxa.deg2raw(self.goal_position_deg[3][0])
        self.goal_position_raw[3][1] = self.rh_femur.deg2raw(self.goal_position_deg[3][1])
        self.goal_position_raw[3][2] = self.rh_tibia.deg2raw(self.goal_position_deg[3][2])


    def setPointCalibration(self, data):
        """adding calibration to position servo refered to actual position"""
        self.goal_position_deg[0][0] = self.position_adder[0][0] + data[0][0]
        self.goal_position_deg[0][1] = self.position_adder[0][1] + data[0][1]
        self.goal_position_deg[0][2] = self.position_adder[0][2] + data[0][2]

        self.goal_position_deg[1][0] = self.position_adder[1][0] + data[1][0]
        self.goal_position_deg[1][1] = self.position_adder[1][1] + data[1][1]
        self.goal_position_deg[1][2] = self.position_adder[1][2] + data[1][2]

        self.goal_position_deg[2][0] = self.position_adder[2][0] + data[2][0]
        self.goal_position_deg[2][1] = self.position_adder[2][1] + data[2][1]
        self.goal_position_deg[2][2] = self.position_adder[2][2] + data[2][2]

        self.goal_position_deg[3][0] = self.position_adder[3][0] + data[3][0]
        self.goal_position_deg[3][1] = self.position_adder[3][1] + data[3][1]
        self.goal_position_deg[3][2] = self.position_adder[3][2] + data[3][2]

        self.deg2raw()


    def setGoalPosition(self, data):
        """set goal position\n
           input list with row x column: 4x3
        """
        self.setPointCalibration(data)

        self.lf_coxa.set_goal_position(self.goal_position_raw[0][0])
        self.lf_femur.set_goal_position(self.goal_position_raw[0][1])
        self.lf_tibia.set_goal_position(self.goal_position_raw[0][2])

        self.lh_coxa.set_goal_position(self.goal_position_raw[1][0])
        self.lh_femur.set_goal_position(self.goal_position_raw[1][1])
        self.lh_tibia.set_goal_position(self.goal_position_raw[1][2])

        self.rf_coxa.set_goal_position(self.goal_position_raw[2][0])
        self.rf_femur.set_goal_position(self.goal_position_raw[2][1])
        self.rf_tibia.set_goal_position(self.goal_position_raw[2][2])

        self.rh_coxa.set_goal_position(self.goal_position_raw[3][0])
        self.rh_femur.set_goal_position(self.goal_position_raw[3][1])
        self.rh_tibia.set_goal_position(self.goal_position_raw[3][2])


    def setInitialPosition(self):
        data = [[150.0, 150.0, 150.0],
                [150.0, 150.0, 150.0],
                [150.0, 150.0, 150.0],
                [150.0, 150.0, 150.0]]

        #femur -0.38 = 21.772396214 derajat 
        #tibia +0.5 = 28.647889755 derajat

        self.setGoalPosition(data)


ax = ServoController()

ax.setGoalPosition()


class Leg(object):
    def __init__(self):
        self.RAD_PER_DEG = 0.017453293
        self.MIN_DEGREE = -11.4592
        # self.MAX_DEGREE = 68.7549 #57.2958
        self.MAX_DEGREE = 97.4 #57.2958
        self.MOVE_STEP = 0.29
        self.MIDDLE_POSITION = 42.975

        self.lf = 0.0
        self.lh = 0.0
        self.rf = 0.0
        self.rh = 0.0
        self.leg_y = 0.0
        self.leg_x = 0.0
        self.last_lf = 0.0
        self.last_lh = 0.0
        self.last_rf = 0.0
        self.last_rh = 0.0
        self.leg_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint', 'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint', 'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint', 'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint']
        
        self.joint_group_publisher = rospy.Publisher('/oped/joint_group_position_controller/command', JointTrajectory, queue_size=1)
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

        # print(self.lf, self.lh, self.rf, self.rh)
        
        self.setPosition(self.lf, self.lh, self.rf, self.rh)


    def setPosition(self, lf_, lh_, rf_, rh_):
        lf, lh, rf, rh = self.degreeToRad(lf_, lh_, rf_, rh_)      

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

        self.leg_position = [lf_hip, lf_upper, lf_lower, lh_hip, lh_upper, lh_lower, rf_hip, rf_upper, rf_lower, rh_hip, rh_upper, rh_lower]
        self.publishPosition()

    
    def setInitialPosition(self):
        self.leg_y = 0.0
        self.leg_x = 0.0
        self.lf = self.MIDDLE_POSITION
        self.lh = self.MIDDLE_POSITION
        self.rf = self.MIDDLE_POSITION
        self.rh = self.MIDDLE_POSITION
        self.setPosition(self.lf, self.lh, self.rf, self.rh)


    def publishPosition(self):
        joints_msg = JointTrajectory()
        joints_msg.header.stamp = rospy.Time.now()
        joints_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0 / 60.0)
        point.positions = self.leg_position #position

        joints_msg.points = [point]
        # rospy.loginfo(joints_msg)
        # rospy.loginfo("---")

        self.joint_group_publisher.publish(joints_msg)


    def getLegPosition(self):
        return self.lf, self.lh, self.rf, self.rh



