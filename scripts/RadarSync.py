import threading, Queue


class RadarEsrSyncThread(threading.Thread):
    def __init__(self, pub_can_send, sync_queue):
	self.pub_can_send = pub_can_send
	self.thread_stop = False
	self.sync_queue  = sync_queue
	threading.Thread.__init__ (self)
   
    def setThreadStop(self, thread_stop):
	self.thread_stop = thread_stop


    def run(self):
	while not self.thread_stop:
	    try:
		val = self.sync_queue.get(True,1)
		self.sync_queue.task_done()
	    except Queue.Empty:
		print 'queue empty from radar_esr sync...'
		pass

    def processEgomotion(self, egomotion_msg):
	self.egomotion = egomotion_msg
