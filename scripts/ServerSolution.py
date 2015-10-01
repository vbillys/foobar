import os
import xmlrpclib
import rospy
import time
import os

def checkRosmasterOn():
	caller_id = '/script'
	m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
	try:
		code, msg, val = m.getSystemState(caller_id)
		return True
	except:
		return False

def startRosmaster():
	try:
		os.system('rosmaster --core &')
		time.sleep(1)
		if not checkRosmasterOn():
			return False
		return True
	except:
		return False


def resolveRosmaster():
	if not checkRosmasterOn():
		# print "server (could) not started! exiting."
		# return 
		try:
			print 'debug: starting server by this self...'
			if not startRosmaster(): raise UserWarning
			print 'luckily succeeded started server'
			return True
		except:
			print 'failed to start server, sorry couldn\'t continue'
			return False
	return True

def resolveParameters(param_name, cmd_retry):
	try:
		_conf = rospy.get_param(param_name)
		return _conf
	except:
		print 'if configuration parameters not available, how can I know which devices to run??'
		# print 'HINT: check if you have run radar_read_param.py yet'
		# return None
		try:
			print 'trying to read parameters...'
			os.system(cmd_retry)
			_conf = rospy.get_param(param_name)
			print 'luckily succeeded read parameters'
			return _conf
		except:
			print 'failed to read parameters, sorry couldn\'t continue'
			return None
