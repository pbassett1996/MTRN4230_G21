#!/usr/bin/env python
import rospy
from mtrn4230_t2_cv.msg import Pick
import random

def talker():
    pub = rospy.Publisher('custom_chatter', Pick)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(0.1) #1hz
    msg = Pick()


    while not rospy.is_shutdown():
        msg.x = random.random()*0.75 + 0.05
        msg.y = random.random()*1.5 - 0.75
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
