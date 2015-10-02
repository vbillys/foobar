#!/usr/bin/env python

import rospy
import can
import threading, Queue
from foobar.msg import Canbus as CanBusMsg
from std_msgs.msg import String

import ServerSolution
import os

# can_interface = 'can0'
# can_interface = 'can1'

class CanSendThread(threading.Thread):
	def __init__ (self, bus):
		self.bus = bus
		threading.Thread.__init__ (self)

	def setThreadStop(self, thread_stop):
		self.thread_stop = thread_stop

	def callback(self, data):
		msg = can.Message(arbitration_id=data.id, data=[i for i in data.data], extended_id=False, dlc=data.dlc)
		self.bus.send(msg)
		
class CanQueuePublishThread(threading.Thread):
	def __init__ (self, can_polling_thread, pub_can_msg, name_id):
		self.can_polling_thread = can_polling_thread
		self.thread_stop = False
		self.counter = 0
		self.canbus_msg = CanBusMsg()
		self.pub_can_msg = pub_can_msg
		self.name_id = name_id
		threading.Thread.__init__ (self)

	def setThreadStop(self, thread_stop):
		self.thread_stop = thread_stop

	def run(self):
		while not self.thread_stop:
			_msg = self.can_polling_thread.getQueue()
			if _msg: 
				self.canbus_msg.header.stamp = rospy.Time.now()
				self.canbus_msg.id = _msg.arbitration_id
				self.canbus_msg.dlc = _msg.dlc
				self.canbus_msg.data= [i for i in _msg.data]
				self.canbus_msg.header.frame_id = self.name_id
				# print type(_msg.data)
				# print type(self.canbus_msg.id), type(_msg.arbitration_id)
				# print format(self.canbus_msg.id,'04x'),format(_msg.arbitration_id,'04x')
				self.pub_can_msg.publish(self.canbus_msg)
				self.counter = self.counter + 1
				# print 'pop from queue', self.counter

	def resetCounter(self):
		self.counter = 0

	def getCounter(self):
		return self.counter


class CanPollingThread(threading.Thread):
	def __init__ (self, can_interface):
		self.bus = can.interface.Bus(can_interface, bustype='socketcan_ctypes')
		self.thread_stop = False
		self.queue = Queue.Queue()
		threading.Thread.__init__ (self)
		
	def getBus(self):
		return self.bus

	def setThreadStop(self, thread_stop):
		self.thread_stop = thread_stop

	def run(self):
		while not self.thread_stop:
			# message = self.bus.recv(0.1)
			message = self.bus.recv(1)
			if message:
				self.queue.put(message)

				# print message

	def getQueue(self):
		try:
			val = self.queue.get(True,1)
			self.queue.task_done()
			return val
		except Queue.Empty:
			return None

		# if not self.queue.empty():
			# val = self.queue.get()
			# self.queue.task_done()
			# return val
		# else:
			# return None


def startCanThread(pub_can_msg, can_interface, name_id):
	can_polling_thread = CanPollingThread(can_interface)
	can_publish_thread = CanQueuePublishThread(can_polling_thread,pub_can_msg, name_id)
	can_send_thread    = CanSendThread(can_polling_thread.getBus())
	can_publish_thread.start()
	can_polling_thread.start()
	# can_send_thread.start()
	return [can_publish_thread, can_polling_thread, can_send_thread]


def getNameResolution(name_resolution, name_id, interface):
	name_res_dict = {
			'name_id': name_id,
			'interface': interface
			}
	return name_res_dict.get(name_resolution, None)

def startRosNode(node_name):

	if not ServerSolution.resolveRosmaster(): return
	# print None
	# print os.path.dirname(os.path.realpath(__file__))
	# devices_conf = ServerSolution.resolveParameters('radar_packet/devices',os.path.dirname(os.path.realpath(__file__))+'radar_read_param.py') 
	# devices_conf = ServerSolution.resolveParameters('radar_packet/devices','radar_read_param.py') 
	devices_conf = ServerSolution.resolveParameters('radar_packet/devices',os.path.dirname(__file__)+'/radar_read_param.py') 
	# devices_conf = ServerSolution.resolveParameters('radar_packet/devices','rosrun foobar radar_read_param.py') 
	if devices_conf is None: return 
	options_conf = ServerSolution.resolveParameters('radar_packet/options',os.path.dirname(__file__)+'/radar_read_param.py') 
	# options_conf = ServerSolution.resolveParameters('radar_packet/options','rosrun foobar radar_read_param.py') 
	if options_conf is None: return 

	if ServerSolution.checkNodeStarted(node_name): return
	rospy.init_node(node_name, anonymous=False)

	print 'start to operating on these devices...'
	for device in devices_conf:
		print device['interface'], device['name_id']

	# return

	thread_list = {}
	pub_can_msg = {}
	pub_log_msg = {}
	sub_can_msg = {}

	pub_log_msg_central  = rospy.Publisher ('radar_packet/log' , String   , queue_size=10)

	for device in devices_conf:
		solved_name_res = getNameResolution(options_conf['name_resolution'],device['name_id'],device['interface'])


		thread_list[device['interface']] = {}
		thread_list[device['interface']]['name_id'] = device['name_id']
		pub_can_msg[device['interface']]  = rospy.Publisher ('radar_packet/'+solved_name_res+'/recv', CanBusMsg, queue_size=10)
		pub_log_msg[device['interface']]  = rospy.Publisher ('radar_packet/'+solved_name_res+'/log' , String   , queue_size=10)
		_thread = startCanThread(pub_can_msg[device['interface']], device['interface'], device['name_id'])
		thread_list[device['interface']]['can_publish_thread'] = _thread[0]
		thread_list[device['interface']]['can_polling_thread'] = _thread[1]
		thread_list[device['interface']]['can_send_thread']    = _thread[2]
		sub_can_msg[device['interface']]  = rospy.Subscriber('radar_packet/'+solved_name_res+'/send', CanBusMsg, thread_list[device['interface']]['can_send_thread'].callback)

		thread_list[device['interface']]['pub_log_msg'] = pub_log_msg[device['interface']]
		print 'started ' + device['interface']

	# print thread_list
	# print pub_can_msg
	# print pub_log_msg
	# print sub_can_msg


	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		# print can_publish_thread.getCounter()
		# pub_log_msg.publish('data received: '+str(can_publish_thread.getCounter()))
		# can_publish_thread.resetCounter()
		_string_log = 'data received: '
		for thread in thread_list:
			_str_counter = str(thread_list[thread]['can_publish_thread'].getCounter())
			thread_list[thread]['pub_log_msg'].publish(_str_counter)
			_string_log = _string_log + thread + ' ' + thread_list[thread]['name_id'] + ' ' + _str_counter + ' '
			thread_list[thread]['can_publish_thread'].resetCounter()
		# print _string_log
		pub_log_msg_central.publish(_string_log)

		rate.sleep()

	print 'stopping canbuses...'
	for thread in thread_list:
		thread_list[thread]['can_publish_thread'].setThreadStop(True)
		thread_list[thread]['can_polling_thread'].setThreadStop(True)
		thread_list[thread]['can_publish_thread'].join()
		thread_list[thread]['can_polling_thread'].join()
		print 'stopped ' + thread
		
		# if thread.isAlive():
			# thread.setThreadStop(True)
			# thread.join()


if __name__ == '__main__':
	try:
		startRosNode('can_wrapper_ros_node')
	except rospy.ROSInterruptException:
		pass

