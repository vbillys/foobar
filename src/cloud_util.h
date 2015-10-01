//#define NDEBUG
#ifndef CLOUD_UTIL_H_CURB_DETECT
#define CLOUD_UTIL_H_CURB_DETECT

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include "common.h"

//#include <exception>


namespace cloud_util {
class CloudUtil
{
protected:
    float min_angle_;
    int   min_index_;
    VAngleData    angle_data_;
    VAngleDataOrgRings angle_data_org_rings_;
    VPointCloud cloud_;
public:
CloudUtil(){};
CloudUtil(VPointCloud::Ptr & cloud)
{
	cloud_.header.stamp    = cloud->header.stamp;
	cloud_.header.frame_id = cloud->header.frame_id;
};
~CloudUtil(){};
void transformCloud( velodyne_rawdata::VPointCloud::Ptr cloud_in
                   , velodyne_rawdata::VPointCloud::Ptr cloud_out
                   , double &x
                   , double &y
                   , double &z
                   , double &theta_x
                   , double &theta_y
                   , double &theta_z
                  );
    
void rangeSelectCloud(velodyne_rawdata::VPointCloud::Ptr &cloud_in
                   , velodyne_rawdata::VPointCloud::Ptr &cloud_out_result
                   , velodyne_rawdata::VPointCloud::Ptr &cloud_out_not_result
                   , std::vector<int8_t> & ring_no //int8_t ring_no 
                   , float height  
                   , float angle_min 
                   , float angle_max 
                  );

void clearFrame(velodyne_rawdata::VPointCloud::Ptr &cloud);

void setCloud(velodyne_rawdata::VPointCloud::Ptr &cloud){
    cloud_.header.stamp    = cloud->header.stamp;
	cloud_.header.frame_id = cloud->header.frame_id;
}

};
}

#endif
