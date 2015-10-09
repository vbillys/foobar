import math
from foobar.msg import Esr_track
from numba import jit, njit
import numba
from bitarray import bitarray
from array import array
import numpy as np

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

def convertRadarToBitVector(msg):
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

# @jit
def convertRadarToArray(msg):
    # b = array('B', 8*[0])
    b = np.array(bytearray(msg.data), dtype=np.uint8)
    # print b.view()
    # msg_bytearray = bytearray(msg.data)
    # index = 0
    # for i in msg.data:
	    # b[index] = msg_bytearray[index]
	    # index = index + 1
    return b


def isSignalSignedType(signal):
    if signal._valuetype == '+':
	return False
    else:
	return True

@njit(numba.i8(numba.u8,numba.u1))
def twosComplement(number, signalsize):
    return (number - (1 << signalsize))

@njit(numba.u1(numba.u1))
def getArrayIdxFromStartBit(n):
    return (0 if (n+1)%8 == 0 else 8-((n+1)%8))  + (n/8)*8

def getFactorIsIntegerFromSignal(signal):
    return True if float(signal._factor).is_integer() else False

def getOffsetIsIntegerFromSignal(signal):
    return True if float(signal._offset).is_integer() else False

# @jit
def getSignalNumber(barray_unpacked, barray, start_bit, signalsize, isByteorderIntel, isValuetypeiSigned, factor, offset):
    # just guard againts unhandled (yet) intel bytecode
    # motorola only for now
    if 1 == isByteorderIntel: raise UserWarning

    # barray_msb = barray[start_bit:start_bit]
    start_bit_idx = getArrayIdxFromStartBit(start_bit)
    barray_msb = barray_unpacked[start_bit_idx]

    start_field_count = (start_bit+1) % 8
    if start_field_count == 0 : start_field_count = 8

    factor_number = int(factor) if float(factor).is_integer() else float(factor)
    offset_number = int(offset) if float(offset).is_integer() else float(offset)

    signal_number_bits = barray_unpacked[start_bit_idx:start_bit_idx+signalsize]

    no_of_padding = signalsize % 8

    signal_number_bits = np.concatenate((np.array(no_of_padding*[0],dtype=np.uint8), signal_number_bits))

    signal_number = np.packbits(signal_number_bits)

    if len(signal_number) < 8:
	signal_number = np.concatenate((signal_number, np.array((8-len(signal_number))*[0],dtype=np.uint8)))
	
    signal_number = signal_number.view(np.uint64).tolist()[0]

    # same field end
    # if signalsize <= start_field_count:
	# signal_number = barray[start_bit:start_bit-signalsize+1]
	# # print type(factor), type(offset)
	# # return signal_number
    # else:
	# barray_map = BitVector(0)
	# end_field_count = (signalsize - start_field_count) % 8
	# field_count     = int(math.floor((signalsize - start_field_count) / 8))
	# end_bit_first_field =int( math.floor(start_bit/8)*8)
	# #first field
	# # print signalsize-1,signalsize-start_field_count,start_bit,end_bit_first_field
	# barray_map[signalsize-1:signalsize-start_field_count] =( barray[start_bit:end_bit_first_field])
	# #intermediary field(s):
	# running_index = end_bit_first_field + 8
	# running_index_map  = signalsize - start_field_count - 1
	# for i in range(0,field_count):
	    # barray_map[running_index_map:running_index_map-7] =( barray[running_index+7:running_index])
	    # running_index = running_index + 8
	    # running_index_map  = running_index_map - 8
	# #last field
	# # check if actually all fields already taken
	# if running_index < 57 and running_index_map >= 0:
	    # barray_map[end_field_count-1:0] = barray[running_index+7:running_index+8-end_field_count]


	# signal_number = barray_map[signalsize-1:0]


    if isValuetypeiSigned and barray_msb: 
	signal_number = twosComplement(signal_number, signalsize)
    return signal_number*factor_number+offset_number


# big endian assumption 
@njit(numba.u8(numba.u1[:],numba.u1,numba.u1))
def getBigEndiNumberFromBitNpArr(blist, idx, size):
    signal_number = 0
    for i in range (0,size):
	signal_number = signal_number | (blist[idx+i] << (size - i))
    return signal_number
    
@njit(numba.u1(numba.u1[:],numba.u1))
def getIsNegativeBigEndianNumberFormBitNpArr(blist, idx):
    return blist[idx]

@njit(numba.f8(numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:]))
def ppParseSignal(barray_unpacked, signal_no, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors ):
    start_bit_idx = getArrayIdxFromStartBit(signal_start_bits[signal_no])
    this_signal_number = getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes[signal_no])
    if signal_is_signed_types[signal_no] and getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
	this_signal_number = twosComplement(this_signal_number, signal_sizes[signal_no])
    this_signal_number = this_signal_number*float(signal_factors[signal_no]) + float(signal_offsets[signal_no])
    return 0

# @jit((numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:]))
# @jit
def pParseSignal(barray_unpacked, no_of_signal, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors ):
    # return [ppParseSignal(barray_unpacked, i, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors ) for i in range(no_of_signal)]
    numbers = []
    for i in range(no_of_signal):
	numbers = numbers + [ppParseSignal(barray_unpacked, i, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors )] 
    return numbers



# @profile
# @jit
def parseSignal(barray, barray_unpacked, signal_names, signals, frame_info):
    signal_list ={} 
    # return None

    #create uint64 from barray
    # msg_data_number = 0
    # for i in range (0,64):
	# msg_data_number = msg_data_number | (barray_unpacked[i] << (i))

    # print  msg_data_number, type(msg_data_number)
    # print twosComplement(0xFFFFFFFF, 32), type(twosComplement(0xFFFFFFFF, 32))
    # print twosComplement(0xFFFFFFFFFFFFFFFF, 64), type(twosComplement(0xFFFFFFFFFFFFFFFF, 64))
    # print twosComplement(0x7FFFFFFF, 32), type(twosComplement(0x7FFFFFF, 32))
    # print twosComplement(0x7FFFFFFFFFFFFFFF, 64), type(twosComplement(0x7FFFFFFFFFFFFFFF, 64))


    # print signal_start_bits, type(signal_start_bits)

    signal_number = pParseSignal(barray_unpacked, len(signal_names)
	    , signal_is_signed_types=frame_info.signal_is_signed_types 
	    , signal_start_bits=frame_info.signal_start_bits 
	    , signal_is_integers=frame_info.signal_is_integers 
	    ,signal_sizes=frame_info.signal_sizes 
	    ,signal_offsets=frame_info.signal_offsets 
	    ,signal_factors=frame_info.signal_factors
	)

    # signal_idx = 0
    # signal_number = []
    # for signal in signals:
	# start_bit_idx = getArrayIdxFromStartBit(signal._startbit)
	# this_signal_number = getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal._signalsize)
	# # print signal_number, type(signal_number), barray_unpacked, type(barray_unpacked)

	# # if isSignalSignedType(signal) and barray_unpacked.tolist()[start_bit_idx]:
	# # if isSignalSignedType(signal) and getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
	# if frame_info.signal_is_signed_types[signal_idx] and getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
	    # this_signal_number = twosComplement(this_signal_number, signal._signalsize)

	# # signal_list[signal._name] = signal_number*signal._factor + signal._offset
	# this_signal_number = this_signal_number*float(signal._factor) + float(signal._offset)
	# # signal_number.append(this_signal_number)
	# signal_number = signal_number + [this_signal_number]
	# signal_idx = signal_idx + 1
	
    # signal_number = [ int(sn) if x else (sn) for (sn,x) in zip(signal_number, frame_info.signal_is_integers)]
    signal_number = []
    for (sn,x) in zip(signal_number, frame_info.signal_is_integers):
	signal_number = signal_number + [ int(sn) if x else (sn)]


    signal_list = dict(zip(signal_names,signal_number))
    # print signal_names
    # print signal_list
    
    # for signal in signal_names:
    # for signal in signals:
	# signal_number = 0
	# start_bit = signal._startbit
	# signalsize = signal._signalsize
	# start_bit_idx = getArrayIdxFromStartBit(start_bit)
	# barray_msb = barray_unpacked[start_bit_idx]

	# start_field_count = (start_bit+1) % 8
	# if start_field_count == 0 : start_field_count = 8

	# signal_number_bits = barray_unpacked[start_bit_idx:start_bit_idx+signalsize]

	# no_of_padding = signalsize % 8

	# signal_number_bits = np.concatenate((np.array(no_of_padding*[0],dtype=np.uint8), signal_number_bits))

	# signal_number = np.packbits(signal_number_bits)

	# if len(signal_number) < 8:
	    # signal_number = np.concatenate((signal_number, np.array((8-len(signal_number))*[0],dtype=np.uint8)))

	# signal_number = signal_number.view(np.uint64).tolist()[0]

	# signal_list[signal._name] = signal_number

    return signal_list
    # return None

# @jit
# @profile
def parseSignal_new(barray, barray_unpacked, signal_names, signals):
    signal_list ={} 
    # for signal in signal_names:
    for signal in signals:
	signal_number = 0
	start_bit = signal._startbit
	signalsize = signal._signalsize
	start_bit_idx = getArrayIdxFromStartBit(start_bit)
	barray_msb = barray_unpacked[start_bit_idx]

	start_field_count = (start_bit+1) % 8
	if start_field_count == 0 : start_field_count = 8

	signal_number_bits = barray_unpacked[start_bit_idx:start_bit_idx+signalsize]

	no_of_padding = signalsize % 8

	signal_number_bits = np.concatenate((np.array(no_of_padding*[0],dtype=np.uint8), signal_number_bits))

	signal_number = np.packbits(signal_number_bits)

	if len(signal_number) < 8:
	    signal_number = np.concatenate((signal_number, np.array((8-len(signal_number))*[0],dtype=np.uint8)))

	signal_number = signal_number.view(np.uint64).tolist()[0]
	
	signal_list[signal._name] = signal_number

    return signal_list


def parseSignal_old(barray, barray_unpacked, signal_names):
    signal_list ={} 
    # return None
    for signal in signal_names:
    # for signal in signals:
	# start_bit = signal._startbit
	# signal_number = barray[start_bit:start_bit]
	# signal_number = getSignalNumber(barray_unpacked, barray, start_bit, signal._signalsize, signal._byteorder, isSignalSignedType(signal), signal._factor, signal._offset)
	signal_number = 0
	# signal_value = signal._values.get(signal_number,None)
	# print signal._name, signal_number, isSignalSignedType(self.test_signal), signal_value, signal._offset, signal._factor
	# signal_list[signal._name] = signal_number
	signal_list[signal] = signal_number

    # print 'hi from parseSignal'
    return signal_list


# @jit(nopython=True)
# @profile
# @jit
def crack(msg_data, signal_names, frame_name, signals, frame_info):
    # barray = convertRadarToBitVector(msg)
    # barray = convertRadarToArray(msg)
    # barray = np.array(bytearray(msg_data), dtype=np.uint8).tolist()
    barray = np.array((msg_data), dtype=np.uint8)
    # barray_unpacked  = np.unpackbits(barray).tolist()
    barray_unpacked  = np.unpackbits(barray)
    # signal = self.test_signal

    # return None

    signal_list = parseSignal(barray.tolist(), barray_unpacked, signal_names, signals, frame_info)
    return signal_list

    # try:
    # print signal_list

@jit
def decodeMsg(msg, signal_list, frame_name):
    result_dict =  decode_dict.get(frame_name, None)
    if result_dict is not None:
	return result_dict(signal_list, msg)
    else:
	return None

def decodeEsrTrack(signal_dict, msg):
    msg_pub = Esr_track()
    msg_pub.grouping_changed  = signal_dict['CAN_TX_TRACK_GROUPING_CHANGED']
    msg_pub.oncoming          = signal_dict['CAN_TX_TRACK_ONCOMING']
    msg_pub.lat_rate          = signal_dict['CAN_TX_TRACK_LAT_RATE']
    msg_pub.bridge_object     = signal_dict['CAN_TX_TRACK_BRIDGE_OBJECT']
    msg_pub.width             = signal_dict['CAN_TX_TRACK_WIDTH']
    msg_pub.status            = signal_dict['CAN_TX_TRACK_STATUS']
    msg_pub.rolling_count     = signal_dict['CAN_TX_TRACK_ROLLING_COUNT']
    msg_pub.range_rate        = signal_dict['CAN_TX_TRACK_RANGE_RATE']
    msg_pub.range_accel       = signal_dict['CAN_TX_TRACK_RANGE_ACCEL']
    msg_pub.range             = signal_dict['CAN_TX_TRACK_RANGE']
    msg_pub.med_range_mode    = signal_dict['CAN_TX_TRACK_MED_RANGE_MODE']
    msg_pub.angle             = signal_dict['CAN_TX_TRACK_ANGLE']
    msg_pub.header.stamp = msg.header.stamp
    msg_pub.header.frame_id = msg.header.frame_id

    # print msg_pub
    
    return msg_pub

decode_dict = {
		'ESR_Track64' : decodeEsrTrack

	    }

