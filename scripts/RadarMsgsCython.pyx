import math
import numpy as np
from foobar.msg  import Esr_track
from cpython cimport array
import array
from libc.stdlib cimport free, malloc
from cpython.mem cimport PyMem_Malloc, PyMem_Realloc, PyMem_Free
cimport cython

def isSignalSignedType(signal):
    if signal._valuetype == '+':
        return False
    else:
        return True

def getFactorIsIntegerFromSignal(signal):
    return True if float(signal._factor).is_integer() else False

def getOffsetIsIntegerFromSignal(signal):
    return True if float(signal._offset).is_integer() else False

ctypedef unsigned char uint8_t
#cdef class FrameInfo:
ctypedef struct FrameInfo:
    unsigned char * signal_is_signed_types
    unsigned char * signal_start_bits
    unsigned char * signal_is_integers
    unsigned char * signal_sizes
    unsigned int howmanysignal
    unsigned int sid
    double * signal_offsets
    double * signal_factors
    #cdef array.array signal_is_signed_types
    #cdef array.array signal_start_bits
    #cdef array.array signal_is_integers
    #cdef array.array signal_sizes
    #cdef array.array signal_offsets
    #cdef array.array signal_factors
#ctypedef struct FrameInfo:
    #unsigned char [:] signal_is_signed_types
    #unsigned char [:] signal_start_bits
    #unsigned char [:] signal_is_integers
    #unsigned char [:] signal_sizes
    #double [:] signal_offsets
    #double [:] signal_factors
#FrameInfo* getthem()

#to hold number of active frames until deallocation (trick to avoid free mem error)
cdef int g_no_of_frames_esr

cdef class ParseCan:
    cdef int value
    cdef int no_of_frames
    cdef FrameInfo *  frame_info
    def __cinit__(self, frame_infos):
        self.value = 12
        cdef int length = len(frame_infos)
        self.no_of_frames = length
        g_no_of_frames_esr = length
        self.frame_info = <FrameInfo *> PyMem_Malloc(length*cython.sizeof(FrameInfo))
        if not self.frame_info:
            print "error Allocating"
        cdef int idx = 0
        #cdef FrameInfo curr_fi
        for frame in frame_infos:
            #curr_fi = self.frame_info[idx]
            #curr_fi.howmanysignal = frame.howmanysignal
            self.frame_info[idx].howmanysignal = frame.howmanysignal
            #print idx, curr_fi.howmanysignal , frame.howmanysignal, self.frame_info[idx].howmanysignal
            self.frame_info[idx].sid = frame.sid
            #self.frame_info[idx].signal_is_signed_types = frame.signal_is_signed_types
            self.frame_info[idx].signal_is_signed_types = <uint8_t*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(uint8_t))
            if not self.frame_info[idx].signal_is_signed_types:
                print "error Allocating"
            #self.frame_info[idx].signal_start_bits = frame.signal_start_bits
            self.frame_info[idx].signal_start_bits = <uint8_t*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(uint8_t))
            if not self.frame_info[idx].signal_start_bits:
                print "error Allocating"
            #self.frame_info[idx].signal_is_integers= frame.signal_is_integers
            self.frame_info[idx].signal_is_integers = <uint8_t*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(uint8_t))
            if not self.frame_info[idx].signal_is_integers:
                print "error Allocating"
            #self.frame_info[idx].signal_sizes = frame.signal_sizes
            self.frame_info[idx].signal_sizes = <uint8_t*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(uint8_t))
            if not self.frame_info[idx].signal_sizes:
                print "error Allocating"
            #self.frame_info[idx].signal_offsets = frame.signal_offsets
            self.frame_info[idx].signal_offsets = <double*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(double))
            if not self.frame_info[idx].signal_offsets:
                print "error Allocating"
            #self.frame_info[idx].signal_factors= frame.signal_factors
            self.frame_info[idx].signal_factors = <double*> PyMem_Malloc(frame.howmanysignal*cython.sizeof(double))
            if not self.frame_info[idx].signal_factors:
                print "error Allocating"
            for l in range(self.frame_info[idx].howmanysignal):
                self.frame_info[idx].signal_is_signed_types[l] = <unsigned char>frame.signal_is_signed_types[l]
                #print self.frame_info[idx].signal_is_signed_types[l] 
                self.frame_info[idx].signal_start_bits[l] = <unsigned char>frame.signal_start_bits[l]
                self.frame_info[idx].signal_is_integers[l] = <unsigned char>frame.signal_is_integers[l]
                self.frame_info[idx].signal_sizes[l] = <unsigned char>frame.signal_sizes[l]
                self.frame_info[idx].signal_offsets[l] = <double>frame.signal_offsets[l]
                self.frame_info[idx].signal_factors[l] = <double>frame.signal_factors[l]
            idx = idx + 1
    def __dealloc__(self):
    #def __del__(self):
        #for idx in range (self.no_of_frames):
        for idx in range (g_no_of_frames_esr):
            PyMem_Free(self.frame_info[idx].signal_is_signed_types)
            PyMem_Free(self.frame_info[idx].signal_start_bits)
            PyMem_Free(self.frame_info[idx].signal_is_integers)
            PyMem_Free(self.frame_info[idx].signal_sizes)
            PyMem_Free(self.frame_info[idx].signal_offsets)
            PyMem_Free(self.frame_info[idx].signal_factors)
        PyMem_Free(self.frame_info)

    cdef test(self):
        return self.value

    #@njit(numba.i8(numba.u8,numba.u1))
    cdef inline long twosComplement(self, unsigned long number, unsigned char signalsize):
        return (number - (1 << signalsize))

    #@njit(numba.u1(numba.u1))
    cdef inline unsigned char getArrayIdxFromStartBit(self, unsigned char n):
        return (0 if (n+1)%8 == 0 else 8-((n+1)%8))  + (n/8)*8


    cdef inline unsigned long getLittleEndiNumberFromBitNpArr(self,unsigned char [:] blist, unsigned char idx, unsigned char size):
        cdef unsigned long signal_number = 0
        cdef unsigned char index 
        #print size, [x for x in blist]
        for i in range (size):
            index = idx + i
            signal_number = signal_number | (blist[index] << i )# (size - i  - 1))
        return signal_number

    # big endian assumption 
    #@njit(numba.u8(numba.u1[:],numba.u1,numba.u1, numba.u4))
    cdef inline unsigned long getBigEndiNumberFromBitNpArr(self,unsigned char [:] blist, unsigned char idx, unsigned char size):
        cdef unsigned long signal_number = 0
        cdef unsigned char index 
        #print size, [x for x in blist]
        for i in range (size):
            index = idx + i
            signal_number = signal_number | (blist[index] << (size - i  - 1))
        return signal_number
        
    #@njit(numba.u1(numba.u1[:],numba.u1))
    cdef inline unsigned char getIsNegativeBigEndianNumberFormBitNpArr(self,unsigned char [:] blist, unsigned char idx):
        return blist[idx]

    #@njit(numba.i4(numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:], numba.u4))
    cdef inline int ppParseSignal(self,barray_unpacked, signal_no, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors , id):
        start_bit_idx = self.getArrayIdxFromStartBit(signal_start_bits[signal_no])
        this_signal_number = self.getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes[signal_no])
        if signal_is_signed_types[signal_no] and self.getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
            this_signal_number = self.twosComplement(this_signal_number, signal_sizes[signal_no])
        return this_signal_number

    cdef inline int ppParseSignalSimple(self,unsigned char [:] barray_unpacked, unsigned char signal_is_signed_types ,unsigned char signal_start_bits ,unsigned char signal_sizes ):
        cdef unsigned char start_bit_idx = self.getArrayIdxFromStartBit(signal_start_bits)
        #print start_bit_idx, signal_start_bits
        #cdef int this_signal_number = 0
        cdef int this_signal_number = self.getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes)
        if signal_is_signed_types and self.getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
            this_signal_number = self.twosComplement(this_signal_number, signal_sizes)
        return this_signal_number

    # @jit((numba.u1[:], numba.u1, numba.u1[:], numba.u1[:], numba.u1[:], numba.u1[:], numba.f8[:], numba.f8[:]))
    # @jit
    cdef inline int[:] pParseSignal(self,unsigned char[:] barray_unpacked, unsigned char no_of_signal, unsigned char[:] signal_is_signed_types ,unsigned char[:] signal_start_bits ,unsigned char[:] signal_sizes ):
        cdef int[:] numbers = array.array('i',[self.ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)])
        #cdef int[:] numbers = [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)]
        #for i in range(no_of_signal):
            ##numbers = numbers + [ppParseSignal(barray_unpacked, i, signal_is_signed_types ,signal_start_bits ,signal_is_integers ,signal_sizes ,signal_offsets ,signal_factors, id)] 
            #numbers = numbers + [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i])] 
        return numbers
        #return [ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)]

    cdef inline int[:] pParseSignal2(self,unsigned char[:] barray_unpacked, int no_of_signal, unsigned char* signal_is_signed_types ,unsigned char* signal_start_bits ,unsigned char* signal_sizes ):
    #cdef inline pParseSignal2(self,unsigned char[:] barray_unpacked, int no_of_signal, unsigned char* signal_is_signed_types ,unsigned char* signal_start_bits ,unsigned char* signal_sizes ):
        #print [(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)]
        cdef int[:] numbers = array.array('i',[self.ppParseSignalSimple(barray_unpacked, signal_is_signed_types[i] ,signal_start_bits[i] ,signal_sizes[i]) for i in range (no_of_signal)])
        return numbers

    def crackScan(self,unsigned char[:]bf_candt, uint8_t[:]bf_count):
        cdef int idx_st
        cdef int idx_ed
        cdef int i, ii
        
        # data structure: ESR_Track01-64, ESR_Valid1-2, ESR_Status1-9, ESR_GroupingIngo (10 times of 0x0540)
        # 64 * 12, 5 * 2, (7, 13, 5, 13, 8, 13, 8, 8, 7), 258 * 10
        cdef array.array esr_numbers = array.array('i', 3440 * [0]) 
        #cdef int [:] esr_track_numbers_view = esr_track_numbers
        #print self.no_of_frames
        for idx in range(self.no_of_frames):
            i = idx
            if bf_count[i] > 0 :
                idx_st = i*64
                idx_ed = i*64 + 64
                #print [x for x in bf_candt[idx_st:idx_ed]], idx_st, idx_ed, i ,(self.frame_info[i].howmanysignal)#, [x for x in self.frame_info[i].signal_is_signed_types],  [x for x in self.frame_info[i].signal_start_bits], [x for x in self.frame_info[i].signal_sizes]

                numbers = self.pParseSignal2(bf_candt[idx_st:idx_ed], 
                #self.pParseSignal2(bf_candt[idx_st:idx_ed], 
                                  self.frame_info[i].howmanysignal,
                                  self.frame_info[i].signal_is_signed_types,
                                  self.frame_info[i].signal_start_bits,
                                  self.frame_info[i].signal_sizes
                                 )
                
                if i >= 4 and i <= 67:
                #if i == 4:
                    ii = i - 4
                    for j in range(12):
                        esr_numbers.data.as_ints[ii*12 + j] = numbers[j]
                        #print numbers[j], esr_track_numbers.data.as_ints[ii*12 + j],  self.frame_info[i].howmanysignal
                elif i == 78 or i ==79:
                    ii = i - 78
                    for j in range(5):
                        esr_numbers.data.as_ints[768+j+(ii*5)] = numbers[j]
                elif i == 0:
                    for j in range(7):
                        esr_numbers.data.as_ints[778+j] = numbers[j]
                elif i == 1:
                    for j in range(13):
                        esr_numbers.data.as_ints[785+j] = numbers[j]
                elif i == 2:
                    for j in range(5):
                        esr_numbers.data.as_ints[798+j] = numbers[j]
                elif i == 3:
                    for j in range(13):
                        esr_numbers.data.as_ints[803+j] = numbers[j]

                elif i == 80:
                    for j in range(8):
                        esr_numbers.data.as_ints[816+j] = numbers[j]
                elif i == 81:
                    for j in range(13):
                        esr_numbers.data.as_ints[824+j] = numbers[j]
                elif i == 82:
                    for j in range(8):
                        esr_numbers.data.as_ints[837+j] = numbers[j]
                elif i == 83:
                    for j in range(8):
                        esr_numbers.data.as_ints[845+j] = numbers[j]
                elif i == 84:
                    for j in range(7):
                        esr_numbers.data.as_ints[853+j] = numbers[j]

                elif i >= 68 and i<= 77:
                    #ii = i - 68
                    ii = numbers[257]
                    for j in range(258):
                        esr_numbers.data.as_ints[854+j+(ii*258)] = numbers[j]



        return esr_numbers
        #return None

    cdef inline int ppParseSignalSimpleSms(self,unsigned char [:] barray_unpacked, unsigned char signal_is_signed_types ,unsigned char signal_start_bits ,unsigned char signal_sizes ):
        #cdef unsigned char start_bit_idx = self.getArrayIdxFromStartBit(signal_start_bits)
        #print start_bit_idx, signal_start_bits
        #cdef int this_signal_number = 0
        #cdef int this_signal_number = self.getBigEndiNumberFromBitNpArr(barray_unpacked, start_bit_idx, signal_sizes)
        cdef int i
        cdef unsigned char barray_unpacked_cc[64]
        for i in range(64):
            barray_unpacked_cc[((i/8)+1)*8 -1  - (i%8)] = barray_unpacked[i]

        #print signal_is_signed_types, signal_start_bits, signal_sizes, barray_unpacked_cc
        cdef int this_signal_number = self.getLittleEndiNumberFromBitNpArr(barray_unpacked_cc, signal_start_bits , signal_sizes)
        #if signal_is_signed_types and self.getIsNegativeBigEndianNumberFormBitNpArr(barray_unpacked, start_bit_idx):
            #this_signal_number = self.twosComplement(this_signal_number, signal_sizes)
        return this_signal_number

    cdef inline int ppParseSignalSimpleSms128(self,unsigned char [:] barray_unpacked, unsigned char signal_is_signed_types ,unsigned char signal_start_bits ,unsigned char signal_sizes ):
        cdef int i
        cdef unsigned char barray_unpacked_cc[128]
        for i in range(128):
            barray_unpacked_cc[((i/8)+1)*8 -1  - (i%8)] = barray_unpacked[i]

        cdef int this_signal_number = self.getLittleEndiNumberFromBitNpArr(barray_unpacked_cc, signal_start_bits , signal_sizes)
        return this_signal_number


    def crackScanSms(self,unsigned char[:]bf_candt,unsigned char[:]bf_raw_target, uint8_t[:]bf_count):
        #cdef int[:] numbers = array.array('i',[self.ppParseSignalSimpleSms(bf_candt[0:64], self.frame_info[0].signal_is_signed_types[i] ,self.frame_info[0].signal_start_bits[i] ,self.frame_info[0].signal_sizes[i]) for i in range (self.frame_info[0].howmanysignal)])
        cdef int[:] numbers
        cdef int[3] raw_target_counts
        cdef array.array sms_numbers = array.array('i', 27 * [0]) 
        #cdef array.array raw_target_numbers
        #cdef int i, ii
        cdef int j, l, k
        cdef int target_cnt, target_idx
        cdef int total_no_of_raw_targets = 0
        #print len(numbers), self.frame_info[0].howmanysignal

        for l in range(3):
                numbers = array.array('i',[self.ppParseSignalSimpleSms(bf_candt[l*64:(l+1)*64], self.frame_info[l].signal_is_signed_types[i] ,self.frame_info[l].signal_start_bits[i] ,self.frame_info[l].signal_sizes[i]) for i in range (self.frame_info[l].howmanysignal)])
                k = 0
                raw_target_counts[l] = numbers[0]
                total_no_of_raw_targets = total_no_of_raw_targets + numbers[0]
                for j in range(l*9,(l+1)*9):
                        #print sms_numbers, numbers, j
                        sms_numbers.data.as_ints[j] = numbers[k]
                        k=k+1
        
        if total_no_of_raw_targets > 0:
            raw_target_numbers = array.array('i', total_no_of_raw_targets * [0] * 6)
            k=0
            for l in range(3):
                for target_cnt in range(raw_target_counts[l]):
                    raw_target_numbers[k*6] = l
                    target_idx = l*128*31+target_cnt*128
                    numbers = array.array('i',[self.ppParseSignalSimpleSms128(bf_raw_target[target_idx:target_idx+128], self.frame_info[3].signal_is_signed_types[i] ,self.frame_info[3].signal_start_bits[i] ,self.frame_info[3].signal_sizes[i]) for i in range (self.frame_info[3].howmanysignal)])
                    for j in range(5):
                        raw_target_numbers[k*6+1+j]= numbers[j]
                    k=k+1

            return sms_numbers +raw_target_numbers
        else:
            return sms_numbers
    # @profile
    # @jit
    def parseSignal(self,unsigned char [:] barray_unpacked, unsigned char [:] signal_sizes, unsigned char [:] signal_start_bits, unsigned char [:] signal_is_signed_types):
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
        return self.pParseSignal(barray_unpacked, len(signal_sizes)
                        , signal_is_signed_types=signal_is_signed_types 
                        , signal_start_bits=signal_start_bits 
                        ,signal_sizes=signal_sizes 
                                                                        )

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

