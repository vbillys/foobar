import threading, Queue
import rospy
import numpy as np

from foobar.msg import Canbus as CanBusMsg

# @njit(numba.u1(numba.u1))
def getArrayIdxFromStartBit(n):
    return (0 if (n+1)%8 == 0 else 8-((n+1)%8))  + (n/8)*8

# @njit(numba.u8(numba.u1[:],numba.u1,numba.u1,numba.u8))
def setBigEndiNumberToNpArr(blist, idx, size, number):
    for i in range (0,size):
	blist[idx+i] = (number & (1 << (size - i  - 1)))

def getFrame(db,frame_name):
    return db._fl.byName(frame_name)

def getFrameById(db, id):
    return db._fl.byId(id)

def getDefaultVehicleData(db, cid, data_list):
    frame = getFrameById(db, cid)
    barray = np.array(8 * [0], dtype=np.uint8)
    barray_unpacked = np.unpackbits(barray)
    idx = 0
    for signal in frame._signals:
	setBigEndiNumberToNpArr(barray_unpacked, getArrayIdxFromStartBit(signal._startbit), signal._signalsize,data_list[idx])
	idx = idx + 1
    data = np.packbits(barray_unpacked).tolist()
    return data

canbus_msg = CanBusMsg()

class RadarEsrSyncThread(threading.Thread):
    def __init__(self, pub_can_send, sync_queue, esr_vehicle_conf, db):
	self.pub_can_send = pub_can_send
	self.thread_stop = False
	self.sync_queue  = sync_queue
	self.sync_counter = 0
	self.esr_vehicle_conf = esr_vehicle_conf
	self.db = db
	self.generateVehicleData()
	threading.Thread.__init__ (self)
   
    def setThreadStop(self, thread_stop):
	self.thread_stop = thread_stop

    
    def generateVehicleData(self):
	self.vehicle1 = CanBusMsg()
	self.vehicle1.id = 0x4f0
	self.vehicle1.dlc = 8
	self.vehicle1.data = getDefaultVehicleData(self.db, self.vehicle1.id, self.esr_vehicle_conf['Vehicle1'])
	self.vehicle2 = CanBusMsg()
	self.vehicle2.id = 0x4f1
	self.vehicle2.dlc = 8
	self.vehicle2.data = getDefaultVehicleData(self.db, self.vehicle2.id, self.esr_vehicle_conf['Vehicle2'])
	self.vehicle3 = CanBusMsg()
	self.vehicle3.id = 0x5f2
	self.vehicle3.dlc = 8
	self.vehicle3.data = getDefaultVehicleData(self.db, self.vehicle3.id, self.esr_vehicle_conf['Vehicle3'])
	self.vehicle4 = CanBusMsg()
	self.vehicle4.id = 0x5f3
	self.vehicle4.dlc = 8
	self.vehicle4.data = getDefaultVehicleData(self.db, self.vehicle4.id, self.esr_vehicle_conf['Vehicle4'])
	self.vehicle5 = CanBusMsg()
	self.vehicle5.id = 0x5f4
	self.vehicle5.dlc = 8
	self.vehicle5.data = getDefaultVehicleData(self.db, self.vehicle5.id, self.esr_vehicle_conf['Vehicle5'])
	self.vehicle6 = CanBusMsg()
	self.vehicle6.id = 0x5f5
	self.vehicle6.dlc = 8
	self.vehicle6.data = getDefaultVehicleData(self.db, self.vehicle6.id, self.esr_vehicle_conf['Vehicle6'])

    def run(self):

	rate = rospy.Rate(50)
	canbus_msg.dlc = 8
	canbus_msg.data = 8 * [0]
	counter = 0

	while not self.thread_stop:
	    # try:
	    #     val = self.sync_queue.get(True,0.02)
	    #     self.sync_queue.task_done()
	    # except Queue.Empty:
	    #     # print 'queue empty from radar_esr sync...'
	    #     pass
	    rate.sleep()
	    if counter == 2:
		self.pub_can_send.publish(self.vehicle1)
		self.pub_can_send.publish(self.vehicle2)
		# for cid in range(0x4f0, 0x4f2):
		    # canbus_msg.id = cid
		    # self.pub_can_send.publish(canbus_msg)
	    elif counter == 5:
		self.pub_can_send.publish(self.vehicle3)
		self.pub_can_send.publish(self.vehicle4)
		self.pub_can_send.publish(self.vehicle5)
		self.pub_can_send.publish(self.vehicle6)
		# for cid in range(0x5f2, 0x5f6):
		    # canbus_msg.id = cid
		    # self.pub_can_send.publish(canbus_msg)

	    counter = counter + 1
	    self.sync_counter = self.sync_counter + 1
	    if counter >= 6:
		counter = 0

    def processEgomotion(self, egomotion_msg):
	self.egomotion = egomotion_msg
