#!/usr/bin/env python

import rospy
import math
from foobar.msg import Canbus as CanBusMsg

import os, sys
import ServerSolution

sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.exportall as ex

class BitVector:
    def __init__(self,val):
        self._val = val

    def __setslice__(self,highIndx,lowIndx,newVal):
        # print newVal, highIndx, lowIndx
        # assert math.ceil(math.log(newVal)/math.log(2)) <= (highIndx-lowIndx+1)

        # clear out bit slice
        clean_mask = (2**(highIndx+1)-1)^(2**(lowIndx)-1)

        self._val = self._val ^ (self._val & clean_mask)
        # set new value
        self._val = self._val | (newVal<<lowIndx)

    def __getslice__(self,highIndx,lowIndx):
        return (self._val>>lowIndx)&(2L**(highIndx-lowIndx+1)-1)

def convertRadarToBitArray(msg):
    b = BitVector(0)
    bit_index = 0
    msg_bytearray = bytearray(msg.data)
    index = 0
    for i in msg.data:
        # b[bit_index+7:bit_index] = ord(i)
        if msg_bytearray[index] != 0:
            b[bit_index+7:bit_index] = msg_bytearray[index]
        bit_index = bit_index + 8
        index = index + 1
    return b


def processRadar(msg):
    if msg.id == 0x04e0:
        # print msg.header.stamp, msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
        # print rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
        print str(msg.header.stamp.secs)+'.'+str(msg.header.stamp.nsecs).rjust(9,'0'), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
        # bytearray(msg.data)
        # print format(convertRadarToBitArray(msg),'016x')
        print convertRadarToBitArray(msg)


def startRosNode(node_name):
    if not ServerSolution.resolveRosmaster(): return
    devices_conf = ServerSolution.resolveParameters('radar_packet/devices','rosrun foobar radar_read_param.py') 
    if devices_conf is None: return 
    if ServerSolution.checkNodeStarted(node_name): return
    rospy.init_node(node_name, anonymous=False)

    rospy.Subscriber('radar_packet', CanBusMsg, processRadar)
    rospy.spin()


if __name__ == '__main__':
    # print os.getcwd()
    # print __file__
    # print os.path.dirname(__file__)

    try:
        startRosNode('radar_preprocess')
    except rospy.ROSInterruptException:
        pass


