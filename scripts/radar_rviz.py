#!/usr/bin/env python

import rospy
import math
import ServerSolution
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from foobar.msg import GenericObjCanData

class RadarVizNode:
    # def startVisuzalizationNode(self,node_name):
    def __init__(self,node_name):
	rospy.init_node(node_name, anonymous=False)
	self.marker = Marker()

	# for i in range (10):
	self.marker.header.frame_id = "/map"
	self.marker.header.stamp = rospy.Time.now()

	self.marker.ns = 'radar_obj'
	self.marker.id = 0 # i

	self.marker_text = Marker()
	self.marker_text.header.frame_id = "/map"
	self.marker_text.header.stamp = rospy.Time.now()
	self.marker_text.ns = 'radar_text'
	self.marker_text.id = 0 # i

	self.marker.type = Marker.CUBE_LIST
	# self.marker.type = Marker.CUBE
	self.marker_text.type =Marker.TEXT_VIEW_FACING

	self.marker.action = Marker.ADD
	self.marker_text.action = Marker.ADD

	self.marker_text.pose.position.x = 0
	self.marker_text.pose.position.y = -5
	self.marker_text.pose.position.z = 0
	self.marker.pose.orientation.x = 0.0
	self.marker.pose.orientation.y = 0.0
	self.marker.pose.orientation.z = 0.0
	self.marker.pose.orientation.w = 1.0
	self.marker_text.pose.orientation.x = 0.0
	self.marker_text.pose.orientation.y = 0.0
	self.marker_text.pose.orientation.z = 0.0
	self.marker_text.pose.orientation.w = 1.0

	self.marker.scale.x = 0.10
	self.marker.scale.y = 0.10
	self.marker.scale.z = 0.10
	self.marker_text.scale.z = 1.10
	self.marker_text.scale.y = 1.10
	self.marker_text.scale.x = 1.10

	self.marker.color.r = 0.0
	self.marker.color.g = 1.0
	self.marker.color.b = 0.0
	self.marker.color.a = 1.0
	self.marker_text.color.r = 0.0
	self.marker_text.color.g = 1.0
	self.marker_text.color.b = 0.0
	self.marker_text.color.a = 1.0

	self.marker_text.text = 'THIS IS AweSome But NODATA yet'

	for x in range (-100,100):
	    point = Point()
	    # point.x = 0
	    point.x = x/10.
	    point.y = 0
	    point.z = 0
	    color = ColorRGBA()
	    color.r = 0.0
	    color.g = 1.0
	    color.b = 0.0
	    color.a = 1.0
	    self.marker.points.append( point)
	    self.marker.colors.append( color)

	self.pub_marker  = rospy.Publisher ('radar_packet/viz/marker' , Marker   , queue_size=10)
	self.pub_marker_text  = rospy.Publisher ('radar_packet/viz/marker_text' , Marker   , queue_size=10)
	solved_name_res = 'can0'
	self.sub_data = rospy.Subscriber ('radar_packet/'+solved_name_res+'/processed', GenericObjCanData, self.onIncomingData)


	# self.pub_marker.publish(self.marker)
	# self.pub_marker_text.publish(self.marker_text)

	self.data_counter = 0

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
	    # pub_marker.publish(marker)
	    # pub_marker.publish(marker_text)
	    # # marker.pose.position.x = marker.pose.position.x + 1
	    
	    self.createTextMarker()
	    if not self.data_counter == 0:
		self.marker_text.text = 'GOT DATA'
	    else:
		self.marker_text.text = 'NODATA'
		self.createCubeMarkerDel()
		self.pub_marker.publish(self.marker)

	    self.data_counter = 0

	    self.pub_marker_text.publish(self.marker_text)

	    rate.sleep()

	# rospy.spin()

    def createCubeMarker(self):
	self.marker = Marker()
	self.marker.header.frame_id = "/map"
	self.marker.header.stamp = rospy.Time.now()
	self.marker.ns = 'radar_obj'
	self.marker.id = 0 # i
	self.marker.action = Marker.ADD
	self.marker.type = Marker.CUBE_LIST
	self.marker.pose.orientation.x = 0.0
	self.marker.pose.orientation.y = 0.0
	self.marker.pose.orientation.z = 0.0
	self.marker.pose.orientation.w = 1.0
	self.marker.scale.x = 0.10
	self.marker.scale.y = 0.10
	self.marker.scale.z = 0.10
	self.marker.color.r = 0.0
	self.marker.color.g = 1.0
	self.marker.color.b = 0.0
	self.marker.color.a = 1.0
	# for x in range (-100,100):
	    # point = Point()
	    # # point.x = 0
	    # point.x = x/10.
	    # point.y = 0
	    # point.z = 0
	    # color = ColorRGBA()
	    # color.r = 0.0
	    # color.g = 1.0
	    # color.b = 0.0
	    # color.a = 1.0
	    # self.marker.points.append( point)
	    # self.marker.colors.append( color)

    def createPointsInMarker(self, data):
	for i in range(0,768,12):
	    point = Point()
	    dist  = data.data[i+9] * .1
	    angle = data.data[i+11]* .1
	    angle_rad = math.radians(angle)
	    point.x = - dist * math.sin(angle_rad)
	    point.y = - dist * math.cos(angle_rad)
	    point.z = 0
	    color = ColorRGBA()
	    color.r = 0.0
	    color.g = 1.0
	    color.b = 0.0
	    color.a = 1.0
	    self.marker.points.append( point)
	    self.marker.colors.append( color)


    def createCubeMarkerDel(self):
	self.marker = Marker()
	self.marker.header.frame_id = "/map"
	self.marker.header.stamp = rospy.Time.now()
	self.marker.ns = 'radar_obj'
	self.marker.id = 0 # i
	self.marker.type = Marker.CUBE
	self.marker.action = Marker.ADD#DELETE
	self.marker.scale.x = 0.10
	self.marker.scale.y = 0.10
	self.marker.scale.z = 0.10

    def createTextMarker(self):
	self.marker_text = Marker()
	self.marker_text.header.frame_id = "/map"
	self.marker_text.header.stamp = rospy.Time.now()
	self.marker_text.ns = 'radar_text'
	self.marker_text.id = 0 # i
	self.marker_text.type =Marker.TEXT_VIEW_FACING
	self.marker_text.action = Marker.ADD
	self.marker_text.pose.position.x = 0
	self.marker_text.pose.position.y = 5
	self.marker_text.pose.position.z = 0
	self.marker_text.pose.orientation.x = 0.0
	self.marker_text.pose.orientation.y = 0.0
	self.marker_text.pose.orientation.z = 0.0
	self.marker_text.pose.orientation.w = 1.0
	self.marker_text.scale.z = 1.10
	self.marker_text.scale.y = 1.10
	self.marker_text.scale.x = 1.10
	self.marker_text.color.r = 0.0
	self.marker_text.color.g = 1.0
	self.marker_text.color.b = 0.0
	self.marker_text.color.a = 1.0

    def onIncomingData(self, msg):
	self.createCubeMarker()
	self.createPointsInMarker(msg)
	self.pub_marker.publish(self.marker)
	self.data_counter = self.data_counter + 1
	# print self.data_counter


if __name__ == '__main__':

    try:
	if not ServerSolution.resolveRosmaster(): raise rospy.ROSInterruptException
	rosnode = RadarVizNode('radar_viz')
	# rosnode.startVisuzalizationNode('radar_viz')
    except rospy.ROSInterruptException:
	pass

