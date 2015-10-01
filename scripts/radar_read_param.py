#!/usr/bin/env python

import rospy
import rospkg
import yaml
from collections import namedtuple


if __name__ == '__main__':
	rospack = rospkg.RosPack()

	print 'Loading conf...'
	
	fo = open(rospack.get_path('foobar')+'/scripts/radar_conf.yaml')
	radar_conf = yaml.load(fo.read())
	fo.close()
	# print radar_conf
	devices_tuple = []
	for device in radar_conf['DEVICES']:
		devices_tuple.append(namedtuple('DeviceTuple',['interface','name_id'])(**device))
	print devices_tuple
	print 'Publishing to param server.'
	rospy.set_param('radar_packet/devices',radar_conf['DEVICES'])
	
