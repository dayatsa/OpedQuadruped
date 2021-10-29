import __future__
from mpu6050 import mpu6050
import math


class Imu(object):
    def __init__(self):
        self.mpu = mpu6050(0x68)
        self.alpha = 0.5

        self.accel_data = [0.0, 0.0, 0.0]
        self.accel_data_filter = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0

        # kalman filter
        self.measurement = [0.0, 0.0]
        self.error_measurement = [3.0,3.0]
        self.estimation = [10.0, 10.0]
        self.error_estimation = [2.0, 2.0]
        self.kalman_gain = [0.0, 0.0]


    def getAccelData(self):
        data = self.mpu.get_accel_data()
        self.accel_data[0] = data['x']
        self.accel_data[1] = data['y']
        self.accel_data[2] = data['z']

    
    def kalmanFilter(self, pitch, roll):
        self.measurement[0] = pitch
        self.measurement[1] = roll

        for i in range(2):
            self.kalman_gain[i] = self.error_estimation[i]/(self.error_estimation[i] + self.error_measurement[i])
            self.estimation[i] = self.estimation[i] + self.kalman_gain[i]*(self.measurement[i] - self.estimation[i])
            self.error_estimation[i] = (1 - self.kalman_gain[i])*self.error_estimation[i]

        print(self.kalman_gain)
        print(self.estimation)
        print(self.error_estimation)
        
        return self.estimation[0], self.estimation[1]

    
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

        return (self.pitch, self.roll)


    def getImuData(self):
        pitch, roll = self.getPitchRoll
        return roll, pitch, 0