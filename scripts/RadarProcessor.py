import sys, os, traceback
import re
sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.importall as im
import math
from foobar.msg import Esr_track
from foobar.msg import GenericObjCanData
from std_msgs.msg import Int32MultiArray
import RadarSync, Queue
import RadarMsgs
import RadarMsgsCython
import RadarProcess
from collections import namedtuple
import numpy as np
from array import array

# import pyximport
# pyximport.install()
# import RadarMsgsCython

def getSignal(frame, signal_name):
    return frame.signalByName(signal_name)

def getFrame(db,frame_name):
    return db._fl.byName(frame_name)

def getFrameById(db, id):
    return db._fl.byId(id)

FrameSignalInfo = namedtuple('FrameSignalInfo','id signal_is_signed_types signal_start_bits signal_is_integers signal_sizes signal_offsets signal_factors howmanysignal sid')
def getProcessedFrameById(db, frame_id):
    frame = getFrameById(db, frame_id)
    return FrameSignalInfo(
	    id = frame_id,
	    signal_is_signed_types  = array('B',[RadarMsgs.isSignalSignedType(signal) for signal in frame._signals]),
	    signal_start_bits  = array('B',[int(signal._startbit) for signal in frame._signals]),
	    signal_is_integers = array('B',[RadarMsgs.getFactorIsIntegerFromSignal(signal) and RadarMsgs.getOffsetIsIntegerFromSignal(signal) for signal in frame._signals]),
	    signal_sizes = array('B',[int(signal._signalsize) for signal in frame._signals]),
	    signal_offsets = array('d',[float(signal._offset) for signal in frame._signals]),
	    signal_factors = array('d',[float(signal._factor) for signal in frame._signals]),
	    howmanysignal = len(frame._signals),
	    sid = frame._Id
	    )
    # return FrameSignalInfo(
	# id = frame_id,
	# signal_is_signed_types  = np.array([RadarMsgs.isSignalSignedType(signal) for signal in frame._signals],dtype=np.uint8),
	# signal_start_bits  = np.array([signal._startbit for signal in frame._signals],dtype=np.uint8),
	# signal_is_integers = np.array([RadarMsgs.getFactorIsIntegerFromSignal(signal) and RadarMsgs.getOffsetIsIntegerFromSignal(signal) for signal in frame._signals],dtype=np.uint8),
	# signal_sizes = np.array([signal._signalsize for signal in frame._signals],dtype=np.uint8),
	# signal_offsets = np.array([signal._offset for signal in frame._signals],dtype=np.float64),
	# signal_factors = np.array([signal._factor for signal in frame._signals],dtype=np.float64),
    # )
    # return FrameSignalInfo(
	# signal_is_signed_types  = [RadarMsgs.isSignalSignedType(signal) for signal in frame._signals],
	# signal_is_signed_types  = np.array(signal_is_signed_types  , dtype=np.uint8),
	# signal_start_bits  = [signal._startbit for signal in signals],
	# signal_start_bits  = np.array(signal_start_bits, dtype=np.uint8),
	# signal_is_integers = [RadarMsgs.getFactorIsIntegerFromSignal(signal) and RadarMsgs.getOffsetIsIntegerFromSignal(signal) for signal in frame._signals],
	# signal_is_integers = np.array(signal_is_integers, dtype=np.uint8),
	# signal_sizes = [signal._signalsize for signal in signals],
	# signal_sizes = np.array(signal_sizes, dtype=np.uint8),
	# signal_offsets = [signal._offset for signal in signals],
	# signal_offsets = np.array(signal_offsets, dtype=np.float64),
	# signal_factors = [signal._factor for signal in signals],
	# signal_factors = np.array(signal_factors, dtype=np.float64)
    # )
    # for signal in frame._signals:
	# signal_data = FrameSignalInfo(
		# is_signed_type = RadarMsgs.isSignalSignedType(signal),
		# is_number_integer = RadarMsgs.getFactorIsIntegerFromSignal(signal) and RadarMsgs.getOffsetIsIntegerFromSignal(signal)
		# )
	# frame_signal_data.append(signal_data)
    # return frame_signal_data

class RadarEsr:
    def __init__(self, pub_can_send, pub_result, name_id, interface, esr_vehicle_conf):


	# self.radar_process_queue = Queue.Queue()#(maxsize=1000)
	# self.radar_process_thread = RadarProcess.RadarEsrProcessThread(pub_result, self.radar_process_queue, self)
	# self.radar_process_thread.start()


	try:
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
		    criteria_ESR_Status = re.match('ESR_Status[0-9]',frame._name)
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
		print [self.registered_processed_dict.get(x).howmanysignal for x in self.registered_processed_dict]
		print [(x).howmanysignal for x in self.registered_processed_frames]
		print [(x).sid for x in self.registered_processed_frames]

		# self.registerFrames(['SODL_Status4','SODL_Status1','SODL_Status2','SODL_Status3'])
		self.counter_processed = 0
		self.msg_pub= GenericObjCanData() 

		self.reg_id_idx = 0
		self.numba_errs = 0
		
		self.track_motion_power_cnt = 0

		# just a trick to prevent multi thread numba complications
		# _siglist = RadarMsgs.crack(8*[0] , [ s._name for s in self.registered_frames[0]._signals] , self.registered_frames[0]._name, self.registered_frames[0]._signals, self.registered_processed_frames[0])
		# _siglist = RadarMsgs.crack(8*[0] , self.registered_processed_frames[0])

		self.first_frame_id = self.registered_Ids[0]
		self.frames_length = len(self.registered_frame_names)
		self.buffered_frames_count = array('B', self.frames_length * [0])
		# self.buffered_frames_canid = array('I', self.frames_length * [0])
		self.buffered_frames_candt = array('B', (self.frames_length * 64) * [0])
		# self.buffered_frames_candt = array('B', 3440 * [0])
		self.parse_can = RadarMsgsCython.ParseCan(self.registered_processed_frames)
		# print self.parse_can.test()

		self.radar_sync_queue = Queue.Queue()
		self.radar_sync_thread = RadarSync.RadarEsrSyncThread(pub_can_send, self.radar_sync_queue, esr_vehicle_conf, self.db, interface)
		self.radar_sync_thread.start()


	except Exception, e:
	    traceback.print_exc(file=sys.stdout)
	    try:
		self.radar_sync_thread.setThreadStop(True)
		self.radar_sync_thread.join()
	    finally:
		raise e

    def resetVehicleData(self, esr_vehicle_conf):
	self.radar_sync_thread.setEsrVehicleConf(esr_vehicle_conf)
	self.radar_sync_thread.generateVehicleData()
	

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

    def getProcessThread(self):
	return self.radar_process_thread

    def registerFrames(self, frame_name_list):
	self.registered_Ids    = sorted([getFrame(self.db, x)._Id for x in frame_name_list])
	# self.registered_frames =        [getFrame(self.db, x)     for x in frame_name_list]
	self.registered_frames =        [getFrameById(self.db, x)     for x in self.registered_Ids]
	# self.registered_processed_frames = [getProcessedFrame(self.db, x) for x in frame_name_list]
	self.registered_processed_frames = [getProcessedFrameById(self.db, x) for x in self.registered_Ids]
	self.registered_processed_dict = dict(zip(self.registered_Ids, self.registered_processed_frames))
	self.registered_dict   = dict(zip(self.registered_Ids, self.registered_frames))
	self.no_of_frame_registered = len(frame_name_list)
	
    def processEgomotion(self, egomotion_msg):
	self.radar_sync_thread.processEgomotion(egomotion_msg)
	
    def DEBUGprintRollingIndexesAndCheck(self,_numbers):
	print '- - - -'

	# print _numbers[825:833]

	print _numbers[782]
	print [_numbers[i] for i in range(768,778)]

	# print [_numbers[783], _numbers[797], _numbers[810]]
	# print [self.buffered_frames_count[i] for i in [0, 1, 3]]
	# if not (_numbers[783] == _numbers[797] and _numbers[797] == _numbers[810]):
	    # print 'rollign count incorrect!'
	# print [_numbers[i] for i in range(6,768,12)]
	# print [self.buffered_frames_count[i] for i in range (4,68)]
	# _thisnumber = _numbers[6]
	# for i in range(6+12,768,12):
	    # if not _numbers[i] == _thisnumber:
		# print 'rolling count not correct!'

	# print [_numbers[i] for i in range(1,768,12)]
	# print [_numbers[i] for i in range(5,768,12)]
	# print [_numbers[i] for i in range(10,768,12)]
	# print [_numbers[i] for i in range(0,768,12)]
	print [_numbers[i] for i in range(4,768,12)]
	# print [_numbers[i] for i in range(3,768,12)]
	print [_numbers[i] for i in range(2,768,12)]
	print [_numbers[i] for i in range(7,768,12)]
	print [_numbers[i] for i in range(8,768,12)]
	# print [_numbers[i] for i in range(9,768,12)]
	# print [_numbers[i] for i in range(11,768,12)]
	# print [_numbers[i], _numbers[i+1], _numbers[i+2], _numbers[i+3], _numbers[i+4], _numbers[i+5], _numbers[i+6] for i in range(854+18,3440,258)]

	# _displaynumbers = []
	# for i in range(854+18,3440-258,258):
	    # _displaynumbers.append(_numbers[i])
	    # _displaynumbers.append(_numbers[i+1])
	    # _displaynumbers.append(_numbers[i+2])
	    # _displaynumbers.append(_numbers[i+3])
	    # _displaynumbers.append(_numbers[i+4])
	    # _displaynumbers.append(_numbers[i+5])
	    # _displaynumbers.append(_numbers[i+6])
	# _displaynumbers.append(_numbers[3440-258+18])
	# print _displaynumbers

	# _displaynumbers = []
	# for i in range(854+11,3440-258,258):
	    # _displaynumbers.append(_numbers[i])
	    # _displaynumbers.append(_numbers[i+1])
	    # _displaynumbers.append(_numbers[i+2])
	    # _displaynumbers.append(_numbers[i+3])
	    # _displaynumbers.append(_numbers[i+4])
	    # _displaynumbers.append(_numbers[i+5])
	    # _displaynumbers.append(_numbers[i+6])
	# _displaynumbers.append(_numbers[3440-258+11])
	# print _displaynumbers

	# _displaynumbers = []
	# for i in range(854+25,3440-258,258):
	    # _displaynumbers.append(_numbers[i])
	    # _displaynumbers.append(_numbers[i+1])
	    # _displaynumbers.append(_numbers[i+2])
	    # _displaynumbers.append(_numbers[i+3])
	    # _displaynumbers.append(_numbers[i+4])
	    # _displaynumbers.append(_numbers[i+5])
	    # _displaynumbers.append(_numbers[i+6])
	# _displaynumbers.append(_numbers[3440-258+25])
	# print _displaynumbers

	# _displaynumbers = []
	# for i in range(854+32,3440-258,258):
	    # _displaynumbers.append(_numbers[i])
	    # _displaynumbers.append(_numbers[i+1])
	    # _displaynumbers.append(_numbers[i+2])
	    # _displaynumbers.append(_numbers[i+3])
	    # _displaynumbers.append(_numbers[i+4])
	    # _displaynumbers.append(_numbers[i+5])
	    # _displaynumbers.append(_numbers[i+6])
	# _displaynumbers.append(_numbers[3440-258+32])
	# print _displaynumbers


	# print [_numbers[i] for i in range(854+228,3440,258)]
	# _thisnumber = _numbers[854+228]
	# for i in range(854+228+258,3440,258):
	    # if not _numbers[i] == _thisnumber:
		# print 'rolling count not correct!'
	# print [self.buffered_frames_count[i] for i in range (68,78)]
	# print [_numbers[i] for i in range(854+257,3440,258)]
	# _thisnumber = _numbers[854+257]
	# for i in range(854+257+258,3440,258):
	    # if not _numbers[i] == _thisnumber+1:
		# print 'group id not correct!'
	    # _thisnumber = _thisnumber + 1

    # @profile
    def processRadar(self,msg):
	# if msg.id == self.first_frame_id:
	    # self.counter_processed = self.counter_processed + 1
	# return
	# print format(msg.id, '04x')
	# if msg.id == self.test_frame._Id: # 0x04e0:

	# frame_detected = self.registered_dict.get(msg.id, None)
	# frame_info = self.registered_processed_dict.get(msg.id, None)
	# print type(msg.data), type(msg.id), type(msg.dlc)

	# if frame_info is not None:
	if msg.id in self.registered_Ids:
	    msg_data = bytearray(msg.data)
	    barray = np.array((msg_data), dtype=np.uint8)
	    barray_unpacked  = np.unpackbits(barray)
	    # barray_unpacked  = array('B',barray_unpacked.tolist())
	    #starting frame
	    if msg.id == self.first_frame_id:
		# if 0 < min(self.track_frame_id.values()) and self.track_frame_id[0x540] >=10:
		    # print '- - - - fullfilled'
		# else:
		    # print '- - - - not completed'
		    # print self.track_frame_id
		# print self.track_frame_id
		# print self.buffered_frames_count
		# print len(self.track_frame_id), len(self.buffered_frames_count), self.frames_length, len(self.registered_Ids)

		# Here we go , push all data in one scan into the decoder
		_numbers = self.parse_can.crackScan(self.buffered_frames_candt,
						   self.buffered_frames_count
						  )

		self.msg_pub.header.stamp = msg.header.stamp
		self.msg_pub.header.frame_id = msg.header.frame_id

		self.msg_pub.data = _numbers

		# print msg_pub[853:860]
		# print msg_pub[768:773]
		# if msg_pub is not None:
		self.pub_result.publish(self.msg_pub)

		# print msg.header.stamp

		self.DEBUGprintRollingIndexesAndCheck(_numbers)

		self.radar_sync_thread.setSyncRollingCount(_numbers[782])

		# self.track_frame_id = dict.fromkeys(self.registered_Ids, 0)
		# self.track_frame_id[self.registered_Ids[0]] = 1
		self.counter_processed = self.counter_processed + 1

		self.buffered_frames_candt = array('B', (self.frames_length * 64) * [0])
		# self.buffered_frames_candt = array('B', 3440 * [0])
		self.buffered_frames_count= array('B', self.frames_length * [0])
		self.buffered_frames_count[0] = 1
		for i in range(64):
		    self.buffered_frames_candt[i] = barray_unpacked[i]
		self.track_motion_power_cnt = 0

	    elif msg.id == 0x0540: #ESR_TrackMotionPower need special attention:
		# we have a good faith here, that this ESR_TrackMotionPower comes in 10 per set
		buffered_index = self.registered_Ids.index(msg.id)
		if self.track_motion_power_cnt <= 10:
		    self.track_motion_power_cnt = self.track_motion_power_cnt + 1
		buffered_index = buffered_index + self.track_motion_power_cnt - 1
		self.buffered_frames_count[buffered_index] = self.buffered_frames_count[buffered_index] + 1
		for i in range(64):
		    self.buffered_frames_candt[i+(buffered_index*64)] = barray_unpacked[i]

	    else:
		# self.track_frame_id[msg.id] = self.track_frame_id[msg.id] + 1
		buffered_index = self.registered_Ids.index(msg.id)
		# print buffered_index
		# print self.buffered_frames_count
		self.buffered_frames_count[buffered_index] = self.buffered_frames_count[buffered_index] + 1
		for i in range(64):
		    # try:
			self.buffered_frames_candt[i+(buffered_index*64)] = barray_unpacked[i]
		    # except:
			# print 'cant assign candt zzz ',  buffered_index, i, len(self.buffered_frames_candt)
		# try:
		    # # self.radar_process_queue.put((msg, frame_detected, frame_info),block=False)
		    # self.radar_process_queue.put(msg, block=False)
		# except Queue.Full:
		    # print "WARNING: process queue full... skipping message"
		    # pass


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


	    # msg_pub = RadarMsgs.crack(msg, frame_detected._signals, frame_detected._name)
	    # _siglist = RadarMsgs.crack(msg, frame_detected._signals, frame_detected._name)
	    # msg_data = msg.data
	    # print msg
	    # print msg_data
	    # print type(msg_data)

	    # msg_pub = None
	    # try:
	    # print frame_info



	    # _siglist = RadarMsgs.crack([x for x in (bytearray(msg.data))] , frame_info, self.parse_can)



	    # _siglist = RadarMsgs.crack([x for x in (bytearray(msg.data))] , [ s._name for s in frame_detected._signals] , frame_detected._name, frame_detected._signals, frame_info)
	    # _siglist = RadarMsgsCython.crack([x for x in (bytearray(msg.data))] , [ s._name for s in frame_detected._signals] , frame_detected._name, frame_detected._signals, frame_info)
	    # if msg.id == 0x5e4:
		# print _siglist
	    # msg_pub = RadarMsgs.decodeMsg(msg, _siglist, frame_detected._name) 

	    # except AttributeError:
		# self.numba_errs = self.numba_errs + 1
		# print 'Attribute Error attributed ' , self.numba_errs



	    # print msg_pub




	    # self.counter_processed = self.counter_processed + 1
	    # except:
		# pass


	    # msg_pub = decodeEsrTrack(signal_list)

	    # print self.counter_processed
	    # print '- - - -'
	# else:
	    # print format(msg.id, '04x') + ' not registered'


