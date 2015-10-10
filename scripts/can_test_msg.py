#!/usr/bin/env python

import rospy
from foobar.msg import Canbus as CanBusMsg
from numba import jit , njit
import numba
import numpy as np

canbus_msg = CanBusMsg()

@njit(numba.u1(numba.u1))
def getArrayIdxFromStartBit(n):
    return (0 if (n+1)%8 == 0 else 8-((n+1)%8))  + (n/8)*8

# @njit(numba.u8(numba.u1[:],numba.u1,numba.u1,numba.u8))
def setBigEndiNumberToNpArr(blist, idx, size, number):
    for i in range (0,size):
	blist[idx+i] = (number & (1 << (size - i  - 1)))
    # return blist

def startRosNode():
    rospy.init_node('can_test_msg_node', anonymous=True)

    # pub_can_msg  = rospy.Publisher('radar_packet/can0/send', CanBusMsg, queue_size=10)
    pub_can_msg  = rospy.Publisher('radar_packet/can0/recv', CanBusMsg, queue_size=100)
    pub_can_msg2 = rospy.Publisher('radar_packet/can1/recv', CanBusMsg, queue_size=100)
    pub_can_msg3 = rospy.Publisher('radar_packet/can2/recv', CanBusMsg, queue_size=100)
    pub_can_msg4 = rospy.Publisher('radar_packet/can3/recv', CanBusMsg, queue_size=100)
    canbus_msg.header.frame_id = 'front_middle'
    #canbus_msg.header.seq = 1
    canbus_msg.id = 0x053f # 0x17be
    canbus_msg.dlc = 8
    # canbus_msg.data = [0x45,255,0,1,5,6,7,9]
    canbus_msg.data = 8 * [0]

    barray = np.array(canbus_msg.data, dtype=np.uint8)
    barray_unpacked = np.unpackbits(barray)
    setBigEndiNumberToNpArr(barray_unpacked, getArrayIdxFromStartBit(55), 8, 1)
    canbus_msg.data = np.packbits(barray_unpacked).tolist()

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
	    pub_can_msg4.publish(canbus_msg)
	canbus_msg.id = 0x540
	for i in range (0,10):
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	    pub_can_msg4.publish(canbus_msg)
	for id in range(0x05d0,0x05d2):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	    pub_can_msg4.publish(canbus_msg)
	for id in range(0x05e4,0x05e9):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	    pub_can_msg4.publish(canbus_msg)
	for id in range(0x04e0,0x04e4):
	    canbus_msg.id = id
	    pub_can_msg.publish(canbus_msg)
	    pub_can_msg2.publish(canbus_msg)
	    pub_can_msg3.publish(canbus_msg)
	    pub_can_msg4.publish(canbus_msg)
	    
        rate.sleep()


if __name__ == '__main__':
    try:
        startRosNode()
    except rospy.ROSInterruptException:
        pass


