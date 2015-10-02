
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

class RadarEsr:
    def __init__(self, pub_can_send, pub_result, name_id, interface):
	self.pub_can_send = pub_can_send
	self.pub_result   = pub_result
	self.name_id      = name_id
	self.interface    = interface

    def processRadar(self,msg):
	if msg.id == 0x04e0:
	    # print msg.header.stamp, msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
	    # print rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [(ord(i)) for i in msg.data]
	    print str(msg.header.stamp.secs)+'.'+str(msg.header.stamp.nsecs).rjust(9,'0'), msg.header.seq, msg.header.frame_id, format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
	    # bytearray(msg.data)
	    # print format(convertRadarToBitArray(msg),'016x')
	    print convertRadarToBitArray(msg)
	    # self.pub_can_send.publish(msg)
	    self.pub_result.publish('from '+self.name_id+' '+self.interface)

