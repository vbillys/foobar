#!/usr/bin/env python

import rospy
import math
from foobar.msg import Canbus as CanBusMsg
from std_msgs.msg import String

import os, sys
import ServerSolution

sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.exportall as ex
import re

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

class RadarEsr:
    def __init__(self, pub_can_send, pub_result, name_id, interface):
        self.pub_can_send = pub_can_send
        self.pub_result   = pub_result
        self.name_id      = name_id
        self.interface    = interface

    def processRadar(self,msg):
        if msg.id == 0x04e0:
            # print msg.header.stamp, msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
            # print rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
            print str(msg.header.stamp.secs)+'.'+str(msg.header.stamp.nsecs).rjust(9,'0'), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
            # bytearray(msg.data)
            # print format(convertRadarToBitArray(msg),'016x')
            print convertRadarToBitArray(msg)
            # self.pub_can_send.publish(msg)
            self.pub_result.publish('from '+self.name_id+' '+self.interface)

def createRadarHandler(radar_list, name_id, interface, radar_type):
    radar_list[name_id] = {}
    pub_dict = {
            'esr': (rospy.Publisher ('radar_packet/'+interface+'/send'     , CanBusMsg, queue_size=10),
                    rospy.Publisher ('radar_packet/'+interface+'/processed', String   , queue_size=10)
                   )
            }
    puber = pub_dict.get(radar_type,None)
    handler_dict = {
            'esr': RadarEsr(puber[0], puber[1], name_id, interface)
            }
    # radar_handler = RadarEsr(puber[0], puber[1], name_id, interface)
    radar_handler = handler_dict.get(radar_type)
    rospy.Subscriber('radar_packet/'+interface+'/recv', CanBusMsg, radar_handler.processRadar)
    radar_list[name_id]['handler'] = radar_handler
    # radar_list[name_id]['pub_can_send'] = handler[0]
    # radar_list[name_id]['pub_result']   = handler[1]

def startRosNode(node_name):
    if not ServerSolution.resolveRosmaster(): return
    devices_conf = ServerSolution.resolveParameters('radar_packet/devices','rosrun foobar radar_read_param.py') 
    if devices_conf is None: return 
    if ServerSolution.checkNodeStarted(node_name): return
    rospy.init_node(node_name, anonymous=False)

    print 'start to read radar packets on these devices...'

    radar_list = {}
    for device in devices_conf:
        _radar_type = 'esr' if re.search('esr',device['name_id']) else None
        print device['interface'], device['name_id'], ' interface: ', _radar_type
        if _radar_type:
            # radar_list[device['name_id']] = {}
            # _pub_can_send  = rospy.Publisher ('radar_packet/'+device['interface']+'/send'     , CanBusMsg, queue_size=10)
            # _pub_result    = rospy.Publisher ('radar_packet/'+device['interface']+'/processed', String, queue_size=10)
            # radar_list[device['name_id']]['pub_can_send'] = _pub_can_send 
            # radar_list[device['name_id']]['pub_result']   = _pub_result

            createRadarHandler(radar_list, device['name_id'], device['interface'], _radar_type)

            # rospy.Subscriber('radar_packet/'+device['interface']+'/recv', CanBusMsg, processRadarEsr)

    # print radar_list
    rospy.spin()


if __name__ == '__main__':
    # print os.getcwd()
    # print __file__
    # print os.path.dirname(__file__)

    try:
        startRosNode('radar_preprocess')
    except rospy.ROSInterruptException:
        pass


