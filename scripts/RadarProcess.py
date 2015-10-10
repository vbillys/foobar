import RadarMsgs
import threading, Queue

class RadarEsrProcessThread(threading.Thread):
    def __init__(self, pub_result, process_queue, radar_processor):
	self.pub_result = pub_result
	self.thread_stop = False
	self.process_queue  = process_queue
	self.process_counter = 0
	self.radar_processor = radar_processor
	threading.Thread.__init__ (self)

    def setThreadStop(self, thread_stop):
	self.thread_stop = thread_stop


    def run(self):
	while not self.thread_stop:
	    try:
		# msg, frame_detected, frame_info = self.process_queue.get(True,1)
		msg = self.process_queue.get(True,1)
		frame_detected = self.radar_processor.registered_dict.get(msg.id, None)
		frame_info = self.radar_processor.registered_processed_dict.get(msg.id, None)
		print (self.process_queue.qsize())
		_siglist = RadarMsgs.crack([x for x in (bytearray(msg.data))] , [ s._name for s in frame_detected._signals] , frame_detected._name, frame_detected._signals, frame_info)
		self.process_counter = self.process_counter + 1
		self.process_queue.task_done()
	    except Queue.Empty:
		print 'queue empty from radar_esr process...'
		pass
