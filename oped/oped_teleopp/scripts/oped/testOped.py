#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String

from Imu import *


imu = Imu()

def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pitch, roll = imu.getPitchRoll()
        print("pitch: {}, roll: {}".format(pitch, roll))

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass