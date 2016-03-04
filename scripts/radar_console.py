#!/usr/bin/env python

import rospy
import math
from foobar.msg import Canbus as CanBusMsg
from foobar.msg import GenericObjCanData


def callbackProcessedRadarData(msg):
    # print msg.data[0], msg.data[9], msg.data[18]
    print msg.data[18], msg.data[9], msg.data[0]

def startRosNode(node_name):
    rospy.init_node(node_name, anonymous=False)
    rospy.Subscriber('radar_packet/group_front_sms/processed', GenericObjCanData, callbackProcessedRadarData)
    rospy.spin()

if __name__ == '__main__':
    try:
        startRosNode('radar_console')
    except rospy.ROSInterruptException:
        pass
