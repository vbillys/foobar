#!/usr/bin/env python

import rospy
import math
from foobar.msg import Canbus as CanBusMsg
from std_msgs.msg import String

import os, sys
import ServerSolution
from RadarProcessor import *

sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.exportall as ex
import re

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


