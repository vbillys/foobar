#!/usr/bin/env python

import rospy
from foobar.msg import Canbus as CanBusMsg

canbus_msg = CanBusMsg()


def startRosNode():
    rospy.init_node('can_test_msg_node', anonymous=True)

    # pub_can_msg  = rospy.Publisher('radar_packet/can0/send', CanBusMsg, queue_size=10)
    pub_can_msg  = rospy.Publisher('radar_packet/can0/recv', CanBusMsg, queue_size=100)
    pub_can_msg2 = rospy.Publisher('radar_packet/can1/recv', CanBusMsg, queue_size=100)
    pub_can_msg3 = rospy.Publisher('radar_packet/can2/recv', CanBusMsg, queue_size=100)
    canbus_msg.header.frame_id = 'front_middle'
    #canbus_msg.header.seq = 1
    canbus_msg.id = 0x053f # 0x17be
    canbus_msg.dlc = 8
    canbus_msg.data = [0x45,255,0,1,5,6,7,9]
    #print len(canbus_msg.data)

    # rate = rospy.Rate(6000)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        canbus_msg.header.stamp = rospy.Time.now()
	# pub_can_msg.publish(canbus_msg)

	for id in range(0x0500,0x0540):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	canbus_msg.id = 0x540
	for i in range (0,10):
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	for id in range(0x05d0,0x05d2):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	for id in range(0x05e4,0x05e9):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	    
        rate.sleep()


if __name__ == '__main__':
    try:
        startRosNode()
    except rospy.ROSInterruptException:
        pass


