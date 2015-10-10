import threading, Queue
import rospy

from foobar.msg import Canbus as CanBusMsg

canbus_msg = CanBusMsg()

class RadarEsrSyncThread(threading.Thread):
    def __init__(self, pub_can_send, sync_queue):
	self.pub_can_send = pub_can_send
	self.thread_stop = False
	self.sync_queue  = sync_queue
	self.sync_counter = 0
	threading.Thread.__init__ (self)
   
    def setThreadStop(self, thread_stop):
	self.thread_stop = thread_stop


    def run(self):

	rate = rospy.Rate(20)
	canbus_msg.dlc = 8
	canbus_msg.data = 8 * [0]
	counter = 0

	while not self.thread_stop:
	    try:
		val = self.sync_queue.get(True,0.02)
		self.sync_queue.task_done()
	    except Queue.Empty:
		# print 'queue empty from radar_esr sync...'
		pass
	    if counter == 2:
		for cid in range(0x4f0, 0x4f2):
		    canbus_msg.id = cid
		    self.pub_can_send.publish(canbus_msg)
	    elif counter == 5:
		for cid in range(0x5f2, 0x5f6):
		    canbus_msg.id = cid
		    self.pub_can_send.publish(canbus_msg)

	    counter = counter + 1
	    self.sync_counter = self.sync_counter + 1
	    if counter >= 6:
		counter = 0

    def processEgomotion(self, egomotion_msg):
	self.egomotion = egomotion_msg
