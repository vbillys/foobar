#!/usr/bin/env python

import rospy
import random

# Messages
from nav_msgs.msg import Odometry

def talker():
	rospy.init_node('egomotion_test_talker', anonymous=True)
	pub  = rospy.Publisher('us_odom', Odometry, queue_size=1)
	rate = rospy.Rate(100) # 10hz
	odometry_msg = Odometry()
	while not rospy.is_shutdown():
		odometry_msg.twist.twist.linear.x = random.random()*15
		odometry_msg.twist.twist.angular.z = random.random()
		pub.publish(odometry_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass



