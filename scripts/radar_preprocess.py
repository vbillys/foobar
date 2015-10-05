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

def getNameResolution(name_resolution, name_id, interface):
    name_res_dict = {
            'name_id': name_id,
            'interface': interface
            }
    return name_res_dict.get(name_resolution, None)

def createRadarHandler(radar_list, name_id, interface, radar_type, name_resolution):
    solved_name_res = getNameResolution(name_resolution, name_id, interface)
    if solved_name_res is None: 
        print 'WARNING: internal error, name resolution faulty, couldni\'t create radar handler'
        return
    radar_list[name_id] = {}
    pub_dict = {
            'esr': (rospy.Publisher ('radar_packet/'+solved_name_res+'/send'     , CanBusMsg, queue_size=10),
                    rospy.Publisher ('radar_packet/'+solved_name_res+'/processed', String   , queue_size=10)
                   )
            }
    puber = pub_dict.get(radar_type,None)
    handler_dict = {
            'esr': RadarEsr(puber[0], puber[1], name_id, interface)
            }
    # radar_handler = RadarEsr(puber[0], puber[1], name_id, interface)
    radar_handler = handler_dict.get(radar_type)
    rospy.Subscriber('radar_packet/'+solved_name_res+'/recv', CanBusMsg, radar_handler.processRadar)
    radar_list[name_id]['handler'] = radar_handler
    radar_list[name_id]['handler_sync_thread'] = radar_handler.getSyncThread()

    # radar_list[name_id]['pub_can_send'] = handler[0]
    # radar_list[name_id]['pub_result']   = handler[1]

def startRosNode(node_name):
    if not ServerSolution.resolveRosmaster(): return
    # devices_conf = ServerSolution.resolveParameters('radar_packet/devices','rosrun foobar radar_read_param.py') 
    devices_conf = ServerSolution.resolveParameters('radar_packet/devices',os.path.dirname(__file__)+'/radar_read_param.py') 
    if devices_conf is None: return 
    # options_conf = ServerSolution.resolveParameters('radar_packet/options','rosrun foobar radar_read_param.py') 
    options_conf = ServerSolution.resolveParameters('radar_packet/options',os.path.dirname(__file__)+'/radar_read_param.py') 
    if options_conf is None: return 
    if ServerSolution.checkNodeStarted(node_name): return
    rospy.init_node(node_name, anonymous=False)

    print 'start to read radar packets on these devices...'
    # print options_conf
    # return



    radar_list = {}
    for device in devices_conf:
        _radar_type = 'esr' if re.search('esr',device['name_id']) else None
        if _radar_type:
            # radar_list[device['name_id']] = {}
            # _pub_can_send  = rospy.Publisher ('radar_packet/'+device['interface']+'/send'     , CanBusMsg, queue_size=10)
            # _pub_result    = rospy.Publisher ('radar_packet/'+device['interface']+'/processed', String, queue_size=10)
            # radar_list[device['name_id']]['pub_can_send'] = _pub_can_send 
            # radar_list[device['name_id']]['pub_result']   = _pub_result

            print device['interface'], device['name_id'], ' interface: ', _radar_type
            createRadarHandler(radar_list, device['name_id'], device['interface'], _radar_type, options_conf['name_resolution'])

            # rospy.Subscriber('radar_packet/'+device['interface']+'/recv', CanBusMsg, processRadarEsr)
        else:
            print device['interface'], device['name_id'], ' interface: WARNING not detected!'

    pub_process_log  = rospy.Publisher ('radar_packet/process_log' , String   , queue_size=10)

    # print radar_list
    # rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        string_log = ''
        for handler in radar_list:
            string_log = string_log + handler + ' ' +  str(radar_list[handler]['handler'].counter_processed) + ' '
        pub_process_log.publish(string_log)
        rate.sleep()

    print "stopping sync thread..."
    for handler in radar_list:
	radar_list[handler]['handler_sync_thread'].setThreadStop(True)
	radar_list[handler]['handler_sync_thread'].join()
	print 'stopped ' + handler



if __name__ == '__main__':
    # print os.getcwd()
    # print __file__
    # print os.path.dirname(__file__)

    try:
        startRosNode('radar_preprocess')
    except rospy.ROSInterruptException:
        pass


