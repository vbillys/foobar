#!/usr/bin/env python

import rospy
import rospkg
import yaml
from collections import namedtuple
import os, sys
from std_srvs.srv import Empty

if __name__ == '__main__':
	rospack = rospkg.RosPack()

	if len(sys.argv) > 1:
	    print 'Loading conf... ' + sys.argv[1]
	    fo = open(os.path.dirname(__file__)+'/' + sys.argv[1])
	else:
	    print 'Loading standard conf...' 
	    # fo = open(rospack.get_path('foobar')+'/scripts/radar_conf.yaml')
	    fo = open(os.path.dirname(__file__)+'/radar_conf.yaml')
	radar_conf = yaml.load(fo.read())
	fo.close()
	# print radar_conf
	devices_tuple = []
	for device in radar_conf['DEVICES']:
		devices_tuple.append(namedtuple('DeviceTuple',['interface','name_id'])(**device))
	print devices_tuple
	print 'topic rendered by ' + radar_conf['OPTIONS']['name_resolution']
	print 'Publishing to param server.'
	rospy.set_param('radar_packet/devices',radar_conf['DEVICES'])
	rospy.set_param('radar_packet/options',radar_conf['OPTIONS'])
	rospy.set_param('radar_packet/esr_vehicle',radar_conf['ESR_VEHICLE'])
	# print [(item,val) for (item,val) in (radar_conf['ESR_VEHICLE'].keys(), radar_conf['ESR_VEHICLE'].values())]
	print [(item,val) for (item,val) in radar_conf['ESR_VEHICLE'].iteritems()]
	print 'trying to reset any ongoing Radar Sync vehicle data'
	try:
	    callable = rospy.ServiceProxy('radar_packet/reread_vehicle_data', Empty)
	    print type(callable())
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

	
