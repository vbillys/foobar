#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

def startVisuzalizationNode(node_name):
    rospy.init_node(node_name, anonymous=False)
    marker = Marker()
    marker_text = Marker()

    # for i in range (10):
    marker.header.frame_id = "/radar_frame"
    marker.header.stamp = rospy.Time.now()
    marker_text.header.frame_id = "/radar_frame"
    marker_text.header.stamp = rospy.Time.now()

    marker.ns = 'radar_obj'
    marker.id = 0 # i
    marker_text.ns = 'radar_text'
    marker_text.id = 0 # i

    marker.type = Marker.CUBE_LIST
    # marker.type = Marker.CUBE
    marker_text.type =Marker.TEXT_VIEW_FACING

    marker.action = Marker.ADD
    marker_text.action = Marker.ADD

    marker_text.pose.position.x = 0
    marker_text.pose.position.y = 0
    marker_text.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker_text.pose.orientation.x = 0.0
    marker_text.pose.orientation.y = 0.0
    marker_text.pose.orientation.z = 0.0
    marker_text.pose.orientation.w = 1.0

    marker.scale.x = 0.10
    marker.scale.y = 0.10
    marker.scale.z = 0.10
    marker_text.scale.z = 1.10
    marker_text.scale.y = 1.10
    marker_text.scale.x = 1.10

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker_text.color.r = 0.0
    marker_text.color.g = 1.0
    marker_text.color.b = 0.0
    marker_text.color.a = 1.0

    marker_text.text = 'THIS IS AweSome'

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
	marker.points.append( point)
	marker.colors.append( color)

    pub_marker  = rospy.Publisher ('radar_packet/viz/marker' , Marker   , queue_size=10)
    pub_marker_text  = rospy.Publisher ('radar_packet/viz/marker_text' , Marker   , queue_size=10)


    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
	pub_marker.publish(marker)
	pub_marker.publish(marker_text)
	# marker.pose.position.x = marker.pose.position.x + 1
	rate.sleep()

if __name__ == '__main__':

    try:
	startVisuzalizationNode('radar_viz')
    except rospy.ROSInterruptException:
	pass

