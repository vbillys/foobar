import sys, os
sys.path.append(os.path.dirname(__file__) + '/canmatrix')
import library.importall as im
import math
from foobar.msg import Esr_track
import RadarSync, Queue

class BitVector:
    def __init__(self,val):
	self._val = val

    def __setslice__(self,highIndx,lowIndx,newVal):
	# print newVal, highIndx, lowIndx
	# assert math.ceil(math.log(newVal)/math.log(2)) <= (highIndx-lowIndx+1)

	# clear out bit slice
	clean_mask = (2**(highIndx+1)-1)^(2**(lowIndx)-1)

	self._val = self._val ^ (self._val & clean_mask)
	# set new value
	self._val = self._val | (newVal<<lowIndx)

    def __getslice__(self,highIndx,lowIndx):
	return (self._val>>lowIndx)&(2L**(highIndx-lowIndx+1)-1)

def convertRadarToBitArray(msg):
    b = BitVector(0)
    bit_index = 0
    msg_bytearray = bytearray(msg.data)
    index = 0
    for i in msg.data:
	# b[bit_index+7:bit_index] = ord(i)
	if msg_bytearray[index] != 0:
	    b[bit_index+7:bit_index] = msg_bytearray[index]
	bit_index = bit_index + 8
	index = index + 1
    return b

def getSignal(frame, signal_name):
    return frame.signalByName(signal_name)

def getFrame(db,frame_name):
    return db._fl.byName(frame_name)

def isSignalSignedType(signal):
    if signal._valuetype == '+':
	return False
    else:
	return True

def twosComplement(number, signalsize):
    return (number - (1 << signalsize))

def getSignalNumber(barray, start_bit, signalsize, isByteorderIntel, isValuetypeiSigned, factor, offset):
    # just guard againts unhandled (yet) intel bytecode
    # motorola only for now
    if 1 == isByteorderIntel: raise UserWarning

    barray_msb = barray[start_bit:start_bit]

    start_field_count = (start_bit+1) % 8
    if start_field_count == 0 : start_field_count = 8

    factor_number = int(factor) if float(factor).is_integer() else float(factor)
    offset_number = int(offset) if float(offset).is_integer() else float(offset)

    # same field end
    if signalsize <= start_field_count:
	signal_number = barray[start_bit:start_bit-signalsize+1]
	# print type(factor), type(offset)
	# return signal_number
    else:
	barray_map = BitVector(0)
	end_field_count = (signalsize - start_field_count) % 8
	field_count     = int(math.floor((signalsize - start_field_count) / 8))
	end_bit_first_field =int( math.floor(start_bit/8)*8)
	#first field
	# print signalsize-1,signalsize-start_field_count,start_bit,end_bit_first_field
	barray_map[signalsize-1:signalsize-start_field_count] =( barray[start_bit:end_bit_first_field])
	#intermediary field(s):
	running_index = end_bit_first_field + 8
	running_index_map  = signalsize - start_field_count - 1
	for i in range(0,field_count):
	    barray_map[running_index_map:running_index_map-7] =( barray[running_index+7:running_index])
	    running_index = running_index + 8
	    running_index_map  = running_index_map - 8
	#last field
	# check if actually all fields already taken
	if running_index < 57 and running_index_map >= 0:
	    barray_map[end_field_count-1:0] = barray[running_index+7:running_index+8-end_field_count]

	signal_number = barray_map[signalsize-1:0]
	

    if isValuetypeiSigned and barray_msb: 
	signal_number = twosComplement(signal_number, signalsize)
    return signal_number*factor_number+offset_number
	    

def decodeEsrTrack(signal_dict):
    msg = Esr_track()
    msg.grouping_changed  = signal_dict['CAN_TX_GROUPING_CHANGED']
    msg.oncoming          = signal_dict['CAN_TX_ONCOMING']
    msg.lat_rate          = signal_dict['CAN_TX_LAT_RATE']
    msg.bridge_object     = signal_dict['CAN_TX_BRIDGE_OBJECT']
    msg.width             = signal_dict['CAN_TX_WIDTH']
    msg.status            = signal_dict['CAN_TX_STATUS']
    msg.rolling_count     = signal_dict['CAN_TX_ROLLING_COUNT']
    msg.range_rate        = signal_dict['CAN_TX_RANGE_RATE']
    msg.range_accel       = signal_dict['CAN_TX_RANGE_ACCEL']
    msg.range             = signal_dict['CAN_TX_RANGE']
    msg.med_range_mode    = signal_dict['CAN_TX_MED_RANGE_MODE']
    msg.angle             = signal_dict['CAN_TX_ANGLE']
    return msg


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

	self.registered_msgs = {}
	self.registerMessage('ESR_Track64',decodeEsrTrack)

    def getSyncThread(self):
	return self.radar_sync_thread

    def registerMessage(self, frame_name, message_func):
	self.registered_msgs[frame_name] = message_func

    def registerFrames(self, frame_name_list):
	self.registered_Ids    = [getFrame(self.db, x)._Id for x in frame_name_list]
	self.registered_frames = [getFrame(self.db, x)     for x in frame_name_list]
	self.registered_dict   = dict(zip(self.registered_Ids, self.registered_frames))
	


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

	    barray = convertRadarToBitArray(msg)
	    # signal = self.test_signal
	    signal_list ={} 
	    for signal in frame_detected._signals:
		start_bit = signal._startbit
		# signal_number = barray[start_bit:start_bit]
		signal_number = getSignalNumber(barray, start_bit, signal._signalsize, signal._byteorder, isSignalSignedType(signal), signal._factor, signal._offset)
		signal_value = signal._values.get(signal_number,None)
		# print signal._name, signal_number, isSignalSignedType(self.test_signal), signal_value, signal._offset, signal._factor
		signal_list[signal._name] = signal_number
	    msg_pub = decodeEsrTrack(signal_list)
	    msg_pub.header.stamp = msg.header.stamp
	    msg.header.frame_id = msg.header.frame_id
	    self.pub_result.publish(msg)
	    self.counter_processed = self.counter_processed + 1
	    # print self.counter_processed
	    # print '- - - -'

	    # print format(msg.id, '04x')

