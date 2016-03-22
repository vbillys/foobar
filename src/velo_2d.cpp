#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub;
ros::Publisher  pub_filtered;

using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
//ros::Publisher  pub, pub_top, pub_bottom, pub_point_filtered;
std::ifstream g_cfg_ifs("velo2d-convert.yaml");
PM::DataPointsFilters g_dpf(g_cfg_ifs);

void processPointCloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  VPointCloud::Ptr cloud(new VPointCloud());
  pcl::fromROSMsg(*cloud_msg , *cloud);
  VPointCloud::Ptr outMsg(new VPointCloud());
  //pcl::PointCloud<pcl::PointXYZ>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZ>());
  outMsg->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
  outMsg->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
  outMsg->height = 1;
  //std::cout << "data arrived.."<< std::endl;
  //float min_z  = 0;
  for (size_t next = 0; next < cloud->points.size(); ++next)
  {
    velodyne_pointcloud::PointXYZIR _point = cloud->points.at(next);
    //if (min_z > _point.z) min_z = _point.z;
    if ( _point.z >= -.41 && _point.z <=.677){ // 7 10


      velodyne_pointcloud::PointXYZIR _point_new;
      //pcl::PointXYZ _point_new;
      _point_new.ring = _point.ring;
      _point_new.x    = _point.x;
      _point_new.y    = _point.y;
      _point_new.z    = 0;//_point.z;
      //_point_new.intensity = (float)25.0;//_point.intensity;
      _point_new.intensity = _point.z;//_point.intensity;
      outMsg->push_back(_point_new);
    }
  }
  //std::cout << min_z << "\n" ;
  sensor_msgs::PointCloud2 point_cloud_total_pcl;//(new PointCloud());
  pcl::toROSMsg(*outMsg, point_cloud_total_pcl);
  //DP mapPointCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(point_cloud_total_pcl));
  DP mapPointCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloud_msg));
  //DP mapPointCloud(outMsg);
  g_dpf.apply(mapPointCloud);
  pub_filtered.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, "velodyne", ros::Time::now()));
  pub.publish(outMsg);
}


  int main(int argc, char** argv) 
  {
    ros::init(argc, argv, "velo_2d_node");
    ros::NodeHandle nh;//("~");
    ros::Rate r(20);

    //ros::Subscriber sub = nh.subscribe ("velodyne_packets", 1 , cloud_cb);
    ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , processPointCloud);

    pub = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points", 1);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_filtered", 1);

    while (ros::ok()){ros::spinOnce();r.sleep();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
  return 0;
}
