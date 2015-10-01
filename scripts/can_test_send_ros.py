#!/usr/bin/env python

import rospy
import can
import threading, Queue

# can_interface = 'can0'
can_interface = 'can1'

class CanSendingThread(threading.Thread):
	def __init__ (self):
		self.bus = can.interface.Bus(can_interface, bustype='socketcan_ctypes')
		self.thread_stop = False
		self.counter = 0
		threading.Thread.__init__ (self)
		
	def setThreadStop(self, thread_stop):
		self.thread_stop = thread_stop

	def resetCounter(self):
		self.counter = 0

	def getCounter(self):
		return self.counter

	def run(self):
		while not self.thread_stop:
			rate = rospy.Rate(1000)
			msg = can.Message(arbitration_id=0xc0ffee, data=[0, 25, 0, 1, 3, 1, 4, 1], extended_id=False)
			# self.bus.send(msg) 
			if self.bus.send(msg) >=0:
				self.counter = self.counter + 1
			# self.counter = self.counter + 1
			rate.sleep()

def startCanSending():
	can_sending_thread = CanSendingThread()
	can_sending_thread.start()
	return can_sending_thread

def startRosNode():
	rospy.init_node('can_test_send_node', anonymous=True)
	_thread = startCanSending()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		print _thread.getCounter()
		_thread.resetCounter()
		rate.sleep()
	_thread.setThreadStop(True)


if __name__ == '__main__':
	try:
		startRosNode()
	except rospy.ROSInterruptException:
		pass
