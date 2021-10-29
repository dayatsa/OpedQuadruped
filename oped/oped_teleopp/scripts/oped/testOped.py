#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String

from Imu import *
from Leg import *


imu = Imu()
oped = Leg()

def talker():


    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # servo.setInitialPosition()
        # oped.addPosition(0,-1)
        # oped.setInitialPosition()

        pitch, roll = imu.getPitchRoll()
        print("pitch: {}, roll: {}".format(pitch, roll))
        print()

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass