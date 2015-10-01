#!/usr/bin/env python

import rospy
from foobar.msg import Canbus as CanBusMsg

canbus_msg = CanBusMsg()


def startRosNode():
    rospy.init_node('can_test_msg_node', anonymous=True)

    pub_can_msg  = rospy.Publisher('radar_packet/can0/send', CanBusMsg, queue_size=10)
    canbus_msg.header.frame_id = 'front_middle'
    #canbus_msg.header.seq = 1
    canbus_msg.id = 0x17be
    canbus_msg.dlc = 8
    canbus_msg.data = [0x45,255,0,1,5,6,7,9]
    #print len(canbus_msg.data)

    rate = rospy.Rate(6000)
    while not rospy.is_shutdown():
        canbus_msg.header.stamp = rospy.Time.now()
        pub_can_msg.publish(canbus_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        startRosNode()
    except rospy.ROSInterruptException:
        pass


