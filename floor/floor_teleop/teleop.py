#!/usr/bin/env python2


from __future__ import print_function

# import roslib; roslib.load_manifest('oped_teleop')
import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


def talker():
    joint1_states_publisher = rospy.Publisher('/floor/joint1_position_controller/command', Float64, queue_size=1)
    joint2_states_publisher = rospy.Publisher('/floor/joint2_position_controller/command', Float64, queue_size=1)
    rospy.init_node('joint_states', anonymous=True)
    # position = np.array([0,0], np.float)
    position = 0
    x = 0.0
    lift = True
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():        
        joints_msg1 = Float64()
        joints_msg2 = Float64()

        # position[0] = x
        # position[1] = x

        x=7

        joints_msg1.data =  x *0.017453293
        joints_msg2.data = 0

        rospy.loginfo("joint1 : " + str(x))
        # rospy.loginfo("joint2 : " + str(joints_msg2))
        rospy.loginfo("---")

        
        if (lift == True):
            x += 5
        else:
            x -= 5
        if(x>=15):
            lift = False
        elif(x<=-15) :
            lift = True    

        # joints_msg.header.stamp = rospy.Time.now()
        # joints_msg.name = ['joint1','joint2']
        # joints_msg.position = position
        
        


        joint1_states_publisher.publish(joints_msg1)
        joint2_states_publisher.publish(joints_msg2)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass