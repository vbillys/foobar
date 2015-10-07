import math


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

def crack(msg, signal_list, signal_name):
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
    try:
	result =  decode_dict.get(signal_name)(signal_list)
	return result
    except:
	return None

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

decode_dict = {
		'ESR_Track64' : decodeEsrTrack

	    }

