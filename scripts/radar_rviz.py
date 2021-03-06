#!/usr/bin/env python

import rospy
import math
import ServerSolution
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from foobar.msg import GenericObjCanData

def extractTrackingFromData(data):
    _movingflags  = []
    _powernumbers = []
    _moveablefast = []
    _moveableslow = []
    for i in range(854+18,3440-258,258):
	_movingflags.append(data.data[i])
	_movingflags.append(data.data[i+1])
	_movingflags.append(data.data[i+2])
	_movingflags.append(data.data[i+3])
	_movingflags.append(data.data[i+4])
	_movingflags.append(data.data[i+5])
	_movingflags.append(data.data[i+6])
	_powernumbers.append(data.data[i  -7])
	_powernumbers.append(data.data[i+1-7])
	_powernumbers.append(data.data[i+2-7])
	_powernumbers.append(data.data[i+3-7])
	_powernumbers.append(data.data[i+4-7])
	_powernumbers.append(data.data[i+5-7])
	_powernumbers.append(data.data[i+6-7])
	_moveableslow.append(data.data[i  +14])
	_moveableslow.append(data.data[i+1+14])
	_moveableslow.append(data.data[i+2+14])
	_moveableslow.append(data.data[i+3+14])
	_moveableslow.append(data.data[i+4+14])
	_moveableslow.append(data.data[i+5+14])
	_moveableslow.append(data.data[i+6+14])
	_moveablefast.append(data.data[i  +7])
	_moveablefast.append(data.data[i+1+7])
	_moveablefast.append(data.data[i+2+7])
	_moveablefast.append(data.data[i+3+7])
	_moveablefast.append(data.data[i+4+7])
	_moveablefast.append(data.data[i+5+7])
	_moveablefast.append(data.data[i+6+7])
    _movingflags.append(data.data[3440-258+18])
    _powernumbers.append(data.data[3440-258+11])
    _moveablefast.append(data.data[3440-258+25])
    _moveableslow.append(data.data[3440-258+32])
    return _movingflags, _powernumbers, _moveablefast, _moveableslow

def extractTargetFromData(data):
    _grouping_changed = [] 
    _oncoming =[]
    _latrate =[]
    _bridgeflag =[]
    _width =[]
    _rangerate =[]
    _rangerateraw =[]
    _rangeaccel =[]
    _range =[]
    _medrangemode =[]
    _angle = []
    _status = []
    for i in range(0,768,12):
	_range.append(data.data[i+9] * .1)
	_angle.append(data.data[i+11]* .1)
	_grouping_changed.append(data.data[i]) 
	_oncoming.append(data.data[i+1])
	_latrate.append(data.data[i+2]*.25)
	_bridgeflag.append(data.data[i+3])
	_width.append(data.data[i+4]*0.5)
	_rangerate.append(data.data[i+7]*.01)
	_rangerateraw.append(data.data[i+7])
	_rangeaccel.append(data.data[i+8]*.05)
	_medrangemode.append(data.data[i+10])
	_status.append(data.data[i+5])
    return _grouping_changed, _oncoming, _latrate, _bridgeflag, _width, _rangerate, _rangeaccel, _range, _medrangemode, _angle, _status, _rangerateraw

class RadarVizNode:
    # def startVisuzalizationNode(self,node_name):
    def __init__(self,node_name, maintain_last_plot):
	rospy.init_node(node_name, anonymous=False)

	self.maintain_last_plot = maintain_last_plot
	# self.createMarkerArray()

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

	self.pub_marker  = rospy.Publisher ('radar_packet/viz/marker' , Marker   , queue_size=100)
	self.pub_marker_text  = rospy.Publisher ('radar_packet/viz/marker_text' , Marker   , queue_size=100)
	self.pub_marker_array = rospy.Publisher ('radar_packet/viz/marker_array' , MarkerArray   , queue_size=1000)
	solved_name_res = 'can0'
	self.sub_data = rospy.Subscriber ('radar_packet/'+solved_name_res+'/processed', GenericObjCanData, self.onIncomingData)


	# self.pub_marker.publish(self.marker)
	# self.pub_marker_text.publish(self.marker_text)

	self.data_counter = 0
	self.filtered_first_track = False

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
	    # pub_marker.publish(marker)
	    # pub_marker.publish(marker_text)
	    # # marker.pose.position.x = marker.pose.position.x + 1
	    
	    # self.createTextMarkerDel()
	    # self.pub_marker_text.publish(self.marker_text)
	    self.createTextMarker()
	    if not self.data_counter == 0:
		self.marker_text.text = 'GOT DATA'
	    else:
		self.marker_text.text = 'NODATA'
		# if self.maintain_last_plot:
		    # self.createCubeMarkerDel()
		    # self.pub_marker.publish(self.marker)
		# self.createCubeMarker()
		# if not self.maintain_last_plot:
		    # self.pub_marker.publish(self.marker)
		# self.createMarkerArray()
		# self.pub_marker_array.publish(self.marker_array)

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
	self.marker.scale.x = 0.50
	self.marker.scale.y = 0.50
	self.marker.scale.z = 0.50
	self.marker.color.r = 0.0
	self.marker.color.g = 1.0
	self.marker.color.b = 0.0
	self.marker.color.a = 1.0
	if not self.maintain_last_plot:
	    self.marker.lifetime = rospy.Duration(1.0)
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
	# track_no_idx = 0
	# for i in range(0,768,12):
	for track_no_idx in self.track_no_idxs:
	# if not self.rangerate[track_no_idx] == 81.91:
	    point = Point()
	    dist  = self.dist [track_no_idx] #data.data[i+9] * .1
	    angle = self.angle[track_no_idx] #data.data[i+11]* .1
	    angle_rad = math.radians(angle)
	    point.x = - dist * math.sin(angle_rad)
	    point.y = - dist * math.cos(angle_rad)
	    point.z = 0
	    color = ColorRGBA()
	    if self.movingflags[track_no_idx]:
		color.r = 1.0
		color.g = 0.0
		color.b = 0.0
	    else:
		color.r = 0.0
		color.g = 1.0
		color.b = 0.0
	    color.a = 1.0
	    self.marker.points.append( point)
	    self.marker.colors.append( color)
	    # track_no_idx = track_no_idx + 1


    def createMarkerArray(self):
	self.marker_array = MarkerArray()
	# for i in range (0,64):
	    # unit = self.spawnTextMarker(i)
	    # self.marker_array.markers.append(unit)


    def createMarkerArrayDel(self):
	self.marker_array = MarkerArray()
	# for id in self.track_no_idxs:
	for id in range(0,64):
	    marker_text = Marker()
	    marker_text.header.frame_id = "/map"
	    marker_text.header.stamp = rospy.Time.now()
	    marker_text.ns = 'radar_array'
	    marker_text.id = id # i
	    marker_text.action = 2 #Marker.DELETEALL
	    self.marker_array.markers.append(marker_text)

    def createTextsInMarkerArray(self, data):
	# track_no_idx = 0
	# print self.track_no_idxs
	# for i in range(0,768,12):
	# for track_no_idx in range(0,64):
	for track_no_idx in self.track_no_idxs:
	    # if not self.rangerate[track_no_idx] == 81.91:
	    dist  = self.dist [track_no_idx] #data.data[i+9] * .1
	    angle = self.angle[track_no_idx] #data.data[i+11]* .1
	    angle_rad = math.radians(angle)
	    unit = self.spawnTextMarker(track_no_idx)
	    # unit = self.marker_array.markers[track_no_idx]
	    # print track_no_idx in self.track_no_idxs

	    unit.text = str(self.powernumbers[track_no_idx] - 10) + ' dB ' + str(self.rangerate[track_no_idx]) + ' ' + str(self.rangeaccel[track_no_idx]) + ' ' + str(self.grouping_changed[track_no_idx]) + ' ' + str(self.width[track_no_idx]) + ' ' + str(self.status[track_no_idx]) + ' ' + str(self.medrangemode[track_no_idx]) + ' ' + str(track_no_idx)


	    # if track_no_idx in self.track_no_idxs:
		# unit.text = str(self.powernumbers[track_no_idx] - 10) + ' dB ' + str(self.rangerate[track_no_idx]) + ' ' + str(self.rangeaccel[track_no_idx]) + ' ' + str(self.grouping_changed[track_no_idx]) + ' ' + str(self.width[track_no_idx]) + ' ' + str(self.status[track_no_idx]) + ' ' + str(track_no_idx)
	    # else:
		# unit.text = ''
	    # print unit.text
	    unit.pose.position.x = - dist * math.sin(angle_rad)
	    unit.pose.position.y = - dist * math.cos(angle_rad) + 0.6
	    self.marker_array.markers.append(unit)
	    # track_no_idx = track_no_idx + 1



    def spawnTextMarker(self, id):
	marker_text = Marker()
	marker_text.header.frame_id = "/map"
	marker_text.header.stamp = rospy.Time.now()
	marker_text.ns = 'radar_array'
	marker_text.id = id # i
	marker_text.type =Marker.TEXT_VIEW_FACING
	marker_text.action = Marker.ADD
	# marker_text.pose.position.x = 0
	# marker_text.pose.position.y = 5
	marker_text.pose.position.z = 0
	marker_text.pose.orientation.x = 0.0
	marker_text.pose.orientation.y = 0.0
	marker_text.pose.orientation.z = 0.0
	marker_text.pose.orientation.w = 1.0
	marker_text.scale.z = 0.60
	marker_text.scale.y = 0.60
	marker_text.scale.x = 0.60
	marker_text.color.r = 1.0
	marker_text.color.g = 0.0
	marker_text.color.b = 1.0
	marker_text.color.a = 1.0
	if not self.maintain_last_plot:
	    marker_text.lifetime = rospy.Duration(1.0)
	return marker_text


    def createCubeMarkerDel(self):
	self.marker = Marker()
	self.marker.header.frame_id = "/map"
	self.marker.header.stamp = rospy.Time.now()
	self.marker.ns = 'radar_obj'
	self.marker.id = 0 # i
	self.marker.type = Marker.CUBE_LIST
	self.marker.action = Marker.DELETE
	# self.marker.scale.x = 0.10
	# self.marker.scale.y = 0.10
	# self.marker.scale.z = 0.10

    def createTextMarkerDel(self):
	self.marker_text = Marker()
	self.marker_text.header.frame_id = "/map"
	self.marker_text.header.stamp = rospy.Time.now()
	self.marker_text.ns = 'radar_text'
	self.marker_text.id = 0 # i
	self.marker_text.action = Marker.DELETE

    def createTextMarker(self):
	self.marker_text = Marker()
	self.marker_text.header.frame_id = "/map"
	self.marker_text.header.stamp = rospy.Time.now()
	self.marker_text.ns = 'radar_text'
	self.marker_text.id = 0 # i
	self.marker_text.type =Marker.TEXT_VIEW_FACING
	self.marker_text.action = Marker.ADD
	self.marker_text.pose.position.x = 0
	self.marker_text.pose.position.y = 1.0
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

    def checkFilterCondition(self, trackid):
	if (self.rangerateraw[trackid] == 8191) or (self.status[trackid] == 0):
	    return False
	else:
	    return True


    def filterTracking(self):
	self.track_no_idxs = []
	for track_no_idx in range (0, 64):
	    if self.checkFilterCondition(track_no_idx):
		self.track_no_idxs.append(track_no_idx)

    def onIncomingData(self, msg):
	self.movingflags, self.powernumbers, self.moveablefast, self.moveableslow = extractTrackingFromData(msg)
	self.grouping_changed, self.oncoming, self.latrate, self.bridgeflag, self.width, self.rangerate, self.rangeaccel, self.dist, self.medrangemode, self.angle, self.status, self.rangerateraw = extractTargetFromData(msg)

	if self.filtered_first_track:
	    self.createMarkerArrayDel()
	    self.pub_marker_array.publish(self.marker_array)
	else:
	    self.filtered_first_track = True

	self.filterTracking()

	# self.createCubeMarkerDel()
	# self.pub_marker.publish(self.marker)
	self.createCubeMarker()
	self.createPointsInMarker(msg)
	self.pub_marker.publish(self.marker)

	self.createMarkerArray()
	self.createTextsInMarkerArray(msg)
	self.pub_marker_array.publish(self.marker_array)

	self.data_counter = self.data_counter + 1
	# print self.data_counter


if __name__ == '__main__':

    try:
	if not ServerSolution.resolveRosmaster(): raise rospy.ROSInterruptException
	rosnode = RadarVizNode('radar_viz', maintain_last_plot = True)
	# rosnode = RadarVizNode('radar_viz', maintain_last_plot = False)
	# rosnode.startVisuzalizationNode('radar_viz')
    except rospy.ROSInterruptException:
	pass

