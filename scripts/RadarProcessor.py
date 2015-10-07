import sys, os
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
	# self.signal       = self.db._fl.byName('SODL_Status1').signalByName('CAN_TX_LOOK_TYPE')
	# self.test_frame   = getFrame(self.db, 'SODL_Status1')
	# self.test_signal  = self.getSignal(self.getFrame('SODL_Status1'), 'CAN_TX_LOOK_TYPE')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_LOOK_TYPE')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_SCAN_INDEX')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_CURVATURE')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_VEHICLE_SPEED_CALC')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_YAW_RATE_CALC')
	# self.test_signal  = getSignal(self.test_frame, 'CAN_TX_DSP_TIMESTAMP')
	# self.registerFrames([ 'SODL_Status1' ])
	# self.registerFrames([ frame._name for frame in self.db._fl._list])
	self.registerFrames(['ESR_Track64'])
	# print 'RadarEsr handler created :', self.registered_dict
	self.counter_processed = 0

	# self.registered_msgs = {}
	# self.registerMessage('ESR_Track64',RadarMsgs.crackEsrTrack)

    def getSyncThread(self):
	return self.radar_sync_thread

    # def registerMessage(self, frame_name, message_func):
	# self.registered_msgs[frame_name] = message_func

    def registerFrames(self, frame_name_list):
	self.registered_Ids    = [getFrame(self.db, x)._Id for x in frame_name_list]
	self.registered_frames = [getFrame(self.db, x)     for x in frame_name_list]
	self.registered_dict   = dict(zip(self.registered_Ids, self.registered_frames))
	
    def processEgomotion(self, egomotion_msg):
	self.RadarEsrSyncThread.processEgomotion(egomotion_msg)
	


    def processRadar(self,msg):
	# if msg.id == self.test_frame._Id: # 0x04e0:
	frame_detected = self.registered_dict.get(msg.id, None)
	if frame_detected is not None:
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

	    try:
		# msg_pub = self.registered_msgs[frame_detected._name](msg, signal_list)
		msg_pub = RadarMsgs.crack(msg, signal_list, frame_detected._name)
		msg_pub.header.stamp = msg.header.stamp
		msg_pub.header.frame_id = msg.header.frame_id
		self.pub_result.publish(msg_pub)
		self.counter_processed = self.counter_processed + 1
	    except:
		pass


	    # msg_pub = decodeEsrTrack(signal_list)

	    # print self.counter_processed
	    # print '- - - -'

	    # print format(msg.id, '04x')

