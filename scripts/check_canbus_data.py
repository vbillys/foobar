#!/usr/bin/env python 
import rospy
from foobar.msg import Canbus as CanBusMsg
import time

pub = rospy.Publisher('/radar_packet/group_front_sms/send', CanBusMsg, queue_size = 10)
# g_send_udt_on = True #False #True#False
g_send_udt_on = False #False #True#False
g_id_udt_on_no = 2
g_tic = time.time()
def processCanbus(msg):
	global g_tic #, g_toc
	if g_send_udt_on:
		activateUDTMsgs(pub)
	# if (msg.id >= 0x700 and msg.id <= 0x70f) or msg.id == 0x500 or msg.id ==0x580 or msg.id == 0x600 or msg.id == 0x680:
	# if msg.id == 0x500 or msg.id ==0x580 or msg.id == 0x600 or msg.id == 0x680:
	# if (msg.id >= 0x700 and msg.id <= 0x70f): # or msg.id == 0x500 or msg.id ==0x580 or msg.id == 0x600 or msg.id == 0x680:
	# if msg.id == 0x501 or msg.id ==0x581 or msg.id == 0x601 or msg.id == 0x681:
	# if msg.id == 0x400 or msg.id ==0x420 or msg.id == 0x440:
	# if (msg.id >= 0x401 and msg.id <= 0x41f) or (msg.id >= 0x421 and msg.id <= 0x43f) or (msg.id >= 0x441 and msg.id <= 0x45f):
	# if (msg.id >= 0x510 and msg.id <= 0x54f) or (msg.id >= 0x590 and msg.id <= 0x5cf) or (msg.id >= 0x610 and msg.id <= 0x64f):
	# if msg.id >= 0x3f5 and msg.id <=0x3f7:
	if msg.id == 0x400 :# or msg.id ==0x420 or msg.id == 0x440:
		_time = time.time()
		print format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
		print _time - g_tic
		g_tic = _time
	if (msg.id >= 0x401 and msg.id <= 0x41f) : #or (msg.id >= 0x421 and msg.id <= 0x43f) or (msg.id >= 0x441 and msg.id <= 0x45f):
		_time = time.time()
		print format(msg.id, '04x'), msg.dlc, [format((ord(i)),'02x') for i in msg.data]
		print _time - g_tic

def activateUDTMsgs(pub):
	canmsg = CanBusMsg()
	canmsg.id = 0x3f2
	canmsg.dlc = 8
	# canmsg.data[1] = 56 #40
	# canmsg.data[4] = 1 #128
	# canmsg.data[0] = 0
	# canmsg.data = [g_id_udt_on_no , 56, 0, 0, 1, 0, 0, 0]
	canmsg.data = [0 , 56, 0, 0, 1, 0, 0, 0]
	pub.publish(canmsg)
	canmsg.data = [1 , 56, 0, 0, 1, 0, 0, 0]
	pub.publish(canmsg)
	canmsg.data = [2 , 56, 0, 0, 1, 0, 0, 0]
	pub.publish(canmsg)


def talker():
	rospy.init_node('check_canbus_data', anonymous=False)

	rospy.Subscriber('/radar_packet/group_front_sms/recv', CanBusMsg, processCanbus)
	# activateUDTMsgs(pub)

	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

