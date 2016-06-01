#!/usr/bin/env python

import rospy
from foobar.msg import Canbus as CanBusMsg

# f_handle = open('canbuslog_esr_1.csv','w')

def processCanMsg(msg):
	str_data = [ "%0.2X" % ord(d) for d in msg.data]
	str_data = ' '.join(str_data)
	print "%0.3X" % msg.id, "%d" % msg.dlc, str_data

def startRosNode():
	rospy.init_node('canbag_to_txt', anonymous=False)
	# rospy.Subscriber('radar_packet', CanBusMsg, processCanMsg)
	rospy.Subscriber('radar_packet/can0/recv', CanBusMsg, processCanMsg)
	rospy.spin()

if __name__ == '__main__':
	try:
		startRosNode()
	except rospy.ROSInterruptException:
		pass
