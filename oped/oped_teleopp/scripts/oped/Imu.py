#!/usr/bin/env python2
import __future__
from mpu6050 import mpu6050
import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class ImuOped(object):
    def __init__(self):
        self.mpu = mpu6050(0x68)
        self.alpha = 0.1

        self.accel_data = [0.0, 0.0, 0.0]
        self.accel_data_filter = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0

        self.LIMIT_UPRIGHT = 1
        self.IMU_MIN_DEGREE = -35
        self.IMU_MAX_DEGREE = 35

        self.pub = rospy.Publisher('/imu_oped/data', Imu, queue_size=1)


    def getAccelData(self):
        data = self.mpu.get_accel_data()
        self.accel_data[0] = data['x']
        self.accel_data[1] = data['y']
        self.accel_data[2] = data['z']

    
    def filterData(self):
        self.accel_data_filter[0] = self.accel_data[0] * self.alpha + (self.accel_data_filter[0] * (1.0 - self.alpha))
        self.accel_data_filter[1] = self.accel_data[1] * self.alpha + (self.accel_data_filter[1] * (1.0 - self.alpha))
        self.accel_data_filter[2] = self.accel_data[2] * self.alpha + (self.accel_data_filter[2] * (1.0 - self.alpha))

    
    def getPitchRoll(self):
        self.getAccelData()
        self.filterData()

        self.pitch  = (math.atan2(-self.accel_data_filter[1], self.accel_data_filter[2])*180.0)/math.pi;
        self.roll = (math.atan2(self.accel_data_filter[0], math.sqrt(self.accel_data_filter[1]*self.accel_data_filter[1] + self.accel_data_filter[2]*self.accel_data_filter[2]))*180.0)/math.pi;
       
        # self.pitch  = (math.atan2(-self.accel_data[1], self.accel_data[2])*180.0)/math.pi;
        # self.roll = (math.atan2(self.accel_data[0], math.sqrt(self.accel_data[1]*self.accel_data[1] + self.accel_data[2]*self.accel_data[2]))*180.0)/math.pi;
        
        # # self.pitch, self.roll 
        # _, _ = self.kalmanFilter(self.pitch, self.roll)
        self.pitch = self.pitch * 1.28082915164085 + 1.94686031049409
        self.pitch = self.pitch * 0.8960 - 0.003761
        self.roll = self.roll * 1.214071024 - 5.05862926724484
        self.roll = self.roll * 0.8890 + 0.8780
        return (self.pitch, self.roll)


    def getImuData(self):
        pitch, roll = self.getPitchRoll()
        # if pitch >= -self.LIMIT_UPRIGHT and pitch <= self.LIMIT_UPRIGHT:
        #     pitch = 0
        # if roll >= -self.LIMIT_UPRIGHT and roll <= self.LIMIT_UPRIGHT:
        #     roll = 0
        return roll, pitch, 0

    
    def talker(self, rt):
        rate = rospy.Rate(rt) # 10hz
        while not rospy.is_shutdown():
            pitch, roll = self.getPitchRoll()

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()

            orientation = Quaternion()
            orientation.x = roll
            orientation.y = pitch
            orientation.z = 0
            orientation.w = 0

            imu_msg.orientation = orientation
            self.pub.publish(imu_msg)
            # rospy.loginfo(imu_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('imu_talker', anonymous=True)
        myImu = ImuOped()
        myImu.talker(50)
    except rospy.ROSInterruptException:
        pass