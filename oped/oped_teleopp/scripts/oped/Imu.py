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

        return (self.pitch, self.roll)