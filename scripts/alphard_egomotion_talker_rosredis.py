#!/usr/bin/env python

import rospy
import redis
import os
import time
import sys
# from __future__ import print_function

# Messages
from nav_msgs.msg import Odometry

# def warning(*objs):
	# print("WARNING: ", *objs, file=sys.stderr)

# current Millisecond function
def current_milli_time():
	"""
	Returns current timestamp in millisecond
	@retval millisecond
	"""
	return int(round(time.time() * 1000))

def redis_start():
	"""
	Redis is Memory stored key->value based database. It used as shared memory.
	Start Redis server and return connection id
	"""
	_dbcon = redis.StrictRedis(host='localhost', port=6379, db=0)
	try:
		dbcon.ping()
		print "Redis Running"
	except:
		print "Redis Starting"
		# os.system('redis-server > redis_log.txt &')
		os.system('redis-server &')
		time.sleep(1)
		try:
			print _dbcon.ping()
			print "Redis Started"
		except:
			print "Redis Can't start'"
			sys.exit(0)
	return _dbcon

def save_to_redis(_data, _dbcon):
	"""
	Save data to REDIS Memory
	"""
	_ts = current_milli_time()
	_dbcon.hmset("EgoMotionFromRos", {"vel_x": _data['vel_x'], "tstamp": _data['tstamp'], "z_angular_rate": _data['z_angular_rate']})
	return True

def callback(data):
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	print data.header
	print data.child_frame_id
	print data.pose 
	print data.twist
	# sys.stderr.write (str(data.header))
	# sys.stderr.write (str(data.child_frame_id))
	# sys.stderr.write (str(data.pose ))
	# sys.stderr.write (str(data.twist))
	# rospy.loginfo (data.header)
	# rospy.loginfo (data.child_frame_id)
	# rospy.loginfo (data.pose)
	# rospy.loginfo (data.twist)

	data_to_redis = {"vel_x": data.twist.twist.linear.x, "tstamp": data.header.seq , "z_angular_rate": data.twist.twist.angular.z}
	save_to_redis(data_to_redis,redis_client)

def listener(redis_client):

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('alphard_egomotion_listener', anonymous=True)

	rospy.Subscriber("us_odom", Odometry, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	try:
		redis_client = redis_start()
		listener(redis_client)
	except rospy.ROSInterruptException:
		pass
