import math
import numpy as np
from foobar.msg  import Esr_track
from cpython cimport array
import array

def isSignalSignedType(signal):
    if signal._valuetype == '+':
        return False
    else:
        return True

#@njit(numba.i8(numba.u8,numba.u1))
cdef inline long twosComplement( unsigned long number, unsigned char signalsize):
    return (number - (1 << signalsize))

#@njit(numba.u1(numba.u1))
cdef inline unsigned char getArrayIdxFromStartBit(unsigned char n):
    return (0 if (n+1)%8 == 0 else 8-((n+1)%8))  + (n/8)*8

def getFactorIsIntegerFromSignal(signal):
    return True if float(signal._factor).is_integer() else False

def getOffsetIsIntegerFromSignal(signal):
    return True if float(signal._offset).is_integer() else False


# big endian assumption 
#@njit(numba.u8(numba.u1[:],numba.u1,numba.u1, numba.u4))
cdef inline unsigned long getBigEndiNumberFromBitNpArr(unsigned char [:] blist, unsigned char idx, unsigned char size):
    cdef unsigned long signal_number = 0
    cdef unsigned char index 
    for i in range (size):
        index = idx + i
        signal_number = signal_number | (blist[index] << (size - i  - 1))
    return signal_number
    
#@njit(numba.u1(numba.u1[:],numba.u1))
cdef inline unsigned char getIsNegativeBigEndianNumberFormBitNpArr(unsigned char [:] blist, unsigned char idx):
    return blist[idx]

#@njit(numba.i4(numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:], numba.u4))
cdef inline int ppParseSignal(barray_unpacked, signal_no, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors , id):
    start_bit_idx = getArrayIdxFromStartBit(signal_start_bits[signal_no])
    this_signal_number = getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes[signal_no])
    if signal_is_signed_types[signal_no] and getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
        this_signal_number = twosComplement(this_signal_number, signal_sizes[signal_no])
    return this_signal_number

cdef inline int ppParseSignalSimple(unsigned char [:] barray_unpacked, unsigned char signal_is_signed_types ,unsigned char signal_start_bits ,unsigned char signal_sizes ):
    cdef unsigned char start_bit_idx = getArrayIdxFromStartBit(signal_start_bits)
    cdef int this_signal_number = getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes)
    if signal_is_signed_types and getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
        this_signal_number = twosComplement(this_signal_number, signal_sizes)
    return this_signal_number

# @jit((numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:]))
# @jit
cdef inline int[:] pParseSignal(unsigned char[:] barray_unpacked, unsigned char no_of_signal, unsigned char[:] signal_is_signed_types ,unsigned char[:] signal_start_bits ,unsigned char[:] signal_sizes ):
    cdef int[:] numbers = array.array('i',[ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)])
    #cdef int[:] numbers = [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)]
    #for i in range(no_of_signal):
        ##numbers = numbers + [ppParseSignal(barray_unpacked, i, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors, id)] 
        #numbers = numbers + [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i])] 
    return numbers
    #return [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)]



# @profile
# @jit
def parseSignal(unsigned char [:] barray_unpacked, unsigned char [:] signal_sizes, unsigned char [:] signal_start_bits, unsigned char [:] signal_is_signed_types):
    #signal_list ={} 

    #signal_number = pParseSignal(barray_unpacked, len(signal_names)
	    #, signal_is_signed_types=frame_info.signal_is_signed_types 
	    #, signal_start_bits=frame_info.signal_start_bits 
	    #, signal_is_integers=frame_info.signal_is_integers 
	    #,signal_sizes=frame_info.signal_sizes 
	    #,signal_offsets=frame_info.signal_offsets 
	    #,signal_factors=frame_info.signal_factors
	    #, id = frame_info.id
	#)

    # if frame_info.id == 0x4e2 or frame_info.id == 0x4e3:
    # if frame_info.id == 0x5d0 or frame_info.id == 0x5d1:
    # if (frame_info.id >= 0x05e4 and frame_info.id <= 0x5e8):
    # if (frame_info.id >= 0x500 and frame_info.id <= 0x510) or frame_info.id == 0x53f:
	# signal_list = dict(zip(signal_names,signal_number))


    # print signal_names
    # print signal_number_converted
	# print signal_list
	# print barray_unpacked
	# print barray
	# print signal_names
	# print signal_number
	# print frame_info.signal_start_bits

    #return signal_list
    return pParseSignal(barray_unpacked, len(signal_sizes)
                    , signal_is_signed_types=signal_is_signed_types 
                    , signal_start_bits=signal_start_bits 
                    ,signal_sizes=signal_sizes 
                                                                        )

cdef class Test:
    cdef public int value
    def __init__(self):
        self.value = 12
    #def test(self):
        #return self.value


class TestInherit(Test):
    def test(self):
        return self.value

# @profile
# @jit
#def crack(msg_data, signal_names, frame_name, signals, frame_info):
    #barray = np.array((msg_data), dtype=np.uint8)
    #barray_unpacked  = np.unpackbits(barray)

    #signal_list = parseSignal(barray.tolist(), barray_unpacked, signal_names, signals, frame_info)
    #return signal_list


# @jit
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
    
    return msg_pub

decode_dict = {
                'ESR_Track64' : decodeEsrTrack
            }

