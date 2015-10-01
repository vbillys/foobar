#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from roslib import message




rospy.init_node('point_cloud_repub', anonymous=True)
pub_cloud = rospy.Publisher("filtered_points", PointCloud2)

def filter_points(msg):
    print 'got point cloud...'
    #pcloud = PointCloud2()
    #pub_cloud.publish(pcloud)
    #pub_cloud.publish(msg)
    print msg.header
    print msg.height
    print msg.width
    print msg.fields
    print msg.is_bigendian
    print msg.point_step
    print msg.row_step
    print msg.is_dense

    

def talker():
    
    #rate = rospy.Rate(100)
    #while not rospy.is_shutdown():
    rospy.spin()


rospy.Subscriber('velodyne_points', PointCloud2, filter_points)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
