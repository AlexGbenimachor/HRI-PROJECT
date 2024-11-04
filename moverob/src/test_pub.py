#!/usr/bin/env python 
import qi
import rospy
from std_msgs.msg import Int32


rospy.init_node("test_pub")
pub  = rospy.Publisher("counter", Int32)
rate = rospy.Rate(2)

count = 0

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
     
