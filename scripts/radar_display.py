#!/usr/bin/env python
import numpy as np
import cv2

import rospy
import math
from foobar.msg import Canbus as CanBusMsg
from foobar.msg import GenericObjCanData


# Create a black image
img_template = np.zeros((512,512,3), np.uint8)

# Draw a diagonal blue line with thickness of 5 px
# img = cv2.line(img,(0,0),(511,511),(255,0,0),5)
cv2.line(img_template,(255,0),(255,511),(255,0,255),1)
cv2.line(img_template,(0,480),(511,480),(255,0,255),1)
# cv2.circle(img, (255,255), 6, (255,0,0),-1)
# cv2.rectangle(img,(255+5,255+5),(255-5,255-5),(0,255,0))

cv2.imshow('Detected Targets',img_template)
cv2.waitKey(10)


def callbackProcessedRadarData(msg):
	# print msg.data[0], msg.data[9], msg.data[18]
	img = np.copy(img_template)
	for i in range (msg.data[9]+msg.data[0]+msg.data[18]):
		radar_id = msg.data[27+i*6]
		r = msg.data[28+i*6]
		a = msg.data[29+i*6]
		r_f = r/1.
		a_f = math.radians((a-511)*.16)
		x_i = int(round(r_f*math.cos(a_f)))
		y_i = int(round(r_f*math.sin(a_f)))
		# if True: #radar_id == 1:
		if radar_id == 1:
			x_i_d = 480 - x_i 
			y_i_d = 255 - y_i
			if x_i_d >=0 and x_i_d <=511 and y_i_d >=0 and y_i_d <=511:
				cv2.rectangle(img,(y_i_d+5,x_i_d+5),(y_i_d-5,x_i_d-5),(0,255,0))

	cv2.imshow('Detected Targets',img)
	cv2.waitKey(10)

def startRosNode(node_name):
	rospy.init_node(node_name, anonymous=False)
	rospy.Subscriber('radar_packet/group_front_sms/processed', GenericObjCanData, callbackProcessedRadarData)
	rospy.spin()

if __name__ == '__main__':
	try:
		startRosNode('radar_display')
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
