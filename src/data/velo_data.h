


#ifndef VELO_DATA_CURB_DETECT
#define VELO_DATA_CURB_DETECT

#include "../data_types.h"
#include <pcl_ros/point_cloud.h>
#include <stdlib.h>
#include "../cloud_util.h"

namespace velodyne_process_data{
	class VLP16PointCloud{
		public:
		    VLP16PointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
		    void transform(EulerTransformParameters &pose);
		    void rangeSelectCloud(
                     std::vector<int8_t> & ring_no 
                   , float height  
                   , float angle_min 
                   , float angle_max 
                  );
		    ~VLP16PointCloud();
		    void convertToRingsData();
		    void convertToRingsDataFromTransformed();
		    void convertToRingsDataFromROI();
		    
		private:
		    
		    void initHeaderPCs();
		    void initHeaderPCsOrg();
		    void initHeader(VPointCloud::Ptr& cloud);
		    void allocateDataMem();
		
		    
		    VPointCloud::Ptr p_raw_data_;
		    VPointCloud::Ptr p_raw_transformed_data_;
		    VPointCloud::Ptr p_roi_data_;
		    VPointCloud::Ptr p_not_roi_data_;
		    
		    OrganizedPointCloud org_data_;
		    
		    
		    EulerTransformParameters transform_;
		    
		    sensor_msgs::PointCloud2ConstPtr& ros_data_;
		    
		    cloud_util::CloudUtil cu_;
	};
}
#endif
