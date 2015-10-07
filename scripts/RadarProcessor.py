import sys, os
import re
sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.importall as im
import math
from foobar.msg import Esr_track
import RadarSync, Queue
import RadarMsgs


def getSignal(frame, signal_name):
    return frame.signalByName(signal_name)

def getFrame(db,frame_name):
    return db._fl.byName(frame_name)

class RadarEsr:
    def __init__(self, pub_can_send, pub_result, name_id, interface):

	self.radar_sync_queue = Queue.Queue()
	self.radar_sync_thread = RadarSync.RadarEsrSyncThread(pub_can_send, self.radar_sync_queue)
	self.radar_sync_thread.start()

	self.pub_can_send = pub_can_send
	self.pub_result   = pub_result
	self.name_id      = name_id
	self.interface    = interface

	# self.db           = im.importDbc(os.path.dirname(__file__)+'/RSDS_PCAN_v18.dbc')
	self.db           = im.importDbc(os.path.dirname(__file__)+'/ESR_radar.dbc')
	# self.registerFrames([ 'SODL_Status1' ])
	# self.registerFrames([ frame._name for frame in self.db._fl._list])
	# self.registerFrames(['ESR_Track64'])

	self.registered_frame_names = []
	for frame in self.db._fl._list:
	    # print frame._name
	    criteria_ESR_Track = re.match('ESR_Track',frame._name)
	    criteria_ESR_Valid = re.match('ESR_Valid',frame._name)
	    criteria_ESR_Status = re.match('ESR_Status[5-9]',frame._name)
	    # print criteria_ESR_Track, criteria_ESR_Valid, criteria_ESR_Status
	    if      (criteria_ESR_Track  is not None) \
		or  (criteria_ESR_Valid  is not None) \
		or  (criteria_ESR_Status is not None):
		    self.registered_frame_names.append(frame._name)

	for i in range (0,9):
	    self.registered_frame_names.append('ESR_TrackMotionPower')
	self.registerFrames(self.registered_frame_names)
	print self.registered_frame_names
	print [format(getFrame(self.db, x)._Id, '04x') for x in self.registered_frame_names]
	self.track_frame_id = dict.fromkeys(self.registered_Ids, 0)
	print self.registered_dict
	print self.track_frame_id

	# self.registerFrames(['SODL_Status4','SODL_Status1','SODL_Status2','SODL_Status3'])
	self.counter_processed = 0

	self.reg_id_idx = 0

    def resetIdIdx(self):
	self.reg_id_idx = 0

    def getIdIdx(self):
	return self.reg_id_idx

    def incrementIdIdx(self):
	if self.reg_id_idx >= self.no_of_frame_registered-1:
	    self.reg_id_idx = 0
	    return True
	else:
	    self.reg_id_idx = self.reg_id_idx + 1
	    return False

    def getSyncThread(self):
	return self.radar_sync_thread

    def registerFrames(self, frame_name_list):
	self.registered_Ids    = sorted([getFrame(self.db, x)._Id for x in frame_name_list])
	self.registered_frames =        [getFrame(self.db, x)     for x in frame_name_list]
	self.registered_dict   = dict(zip(self.registered_Ids, self.registered_frames))
	self.no_of_frame_registered = len(frame_name_list)
	
    def processEgomotion(self, egomotion_msg):
	self.RadarEsrSyncThread.processEgomotion(egomotion_msg)
	


    def processRadar(self,msg):
	# print format(msg.id, '04x')
	# if msg.id == self.test_frame._Id: # 0x04e0:
	frame_detected = self.registered_dict.get(msg.id, None)
	if frame_detected is not None:
	    #starting frame
	    if msg.id == self.registered_Ids[0]:
		if 0 < min(self.track_frame_id.values()) and self.track_frame_id[0x540] >=10:
		    print '- - - - fullfilled'
		else:
		    print '- - - - not completed'
		    print self.track_frame_id
		self.track_frame_id = dict.fromkeys(self.registered_Ids, 0)
		self.track_frame_id[self.registered_Ids[0]] = 1
		self.counter_processed = self.counter_processed + 1
	    else:
		self.track_frame_id[msg.id] = self.track_frame_id[msg.id] + 1


	    # if msg.id == self.registered_Ids[self.getIdIdx()]:
		# is_scan_completed = self.incrementIdIdx()
	    # else:
		# self.resetIdIdx()
		# # print 'reseted id... ' + format(msg.id , '04x')
		# return

	    # print 'is registered'
	    # print format(msg.id, '04x')
	    # if is_scan_completed:
		# print '- - - -'
	    # print frame_detected._name
	    # print msg.header.stamp, msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
	    # print rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
	    # print str(msg.header.stamp.secs)+'.'+str(msg.header.stamp.nsecs).rjust(9,'0'), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
	    # bytearray(msg.data)
	    # print format(convertRadarToBitArray(msg),'016x')
	    # print convertRadarToBitArray(msg)
	    # self.pub_can_send.publish(msg)
	    # self.pub_result.publish('from '+self.name_id+' '+self.interface)

	    # try:
		# msg_pub = self.registered_msgs[frame_detected._name](msg, signal_list)
	    # print 'processing frame,'
	    # print msg


	    msg_pub = RadarMsgs.crack(msg, frame_detected._signals, frame_detected._name)


	    # print msg_pub


	    if msg_pub is not None:
		self.pub_result.publish(msg_pub)


	    # self.counter_processed = self.counter_processed + 1
	    # except:
		# pass


	    # msg_pub = decodeEsrTrack(signal_list)

	    # print self.counter_processed
	    # print '- - - -'
	# else:
	    # print format(msg.id, '04x') + ' not registered'


