#include "cloud_util.h"
#include "debug_print.hpp"
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <initializer_list>

namespace cloud_util {

void CloudUtil::transformCloud(velodyne_rawdata::VPointCloud::Ptr cloud_in
                   , velodyne_rawdata::VPointCloud::Ptr cloud_out
                   , double &x
                   , double &y
                   , double &z
                   , double &theta_x
                   , double &theta_y
                   , double &theta_z
                  )
{
    // Define Eigen matrix(transformaton matrix) rotation about z-axis
    
    // Method II for (transformaton matrix) Affine transformation
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  
    // Define a translation of 2.5 meters on the z axis.
    transform_2.translation() << x, y, z;
  
    // The same rotation matrix as before; tetha radians arround Y axis
    transform_2.rotate (Eigen::AngleAxisf (theta_y, Eigen::Vector3f::UnitY()));
  
    transform_2.rotate (Eigen::AngleAxisf (theta_z, Eigen::Vector3f::UnitZ()));
  
    // The same rotation matrix as before; tetha radians arround X axis
    transform_2.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));
  
    // perform transformation using 
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform_2);  
    
    
}

void CloudUtil::clearFrame(velodyne_rawdata::VPointCloud::Ptr &cloud)
{
	cloud->clear();
	//cloud->header.stamp = cloud_->header.stamp;
	//cloud->header.frame_id = cloud_->header.frame_id;
	cloud->header.stamp = cloud_.header.stamp;
	cloud->header.frame_id = cloud_.header.frame_id;
	cloud->height = 1;
}

void CloudUtil::rangeSelectCloud(velodyne_rawdata::VPointCloud::Ptr &cloud_in
                   , velodyne_rawdata::VPointCloud::Ptr &cloud_out_result
                   , velodyne_rawdata::VPointCloud::Ptr &cloud_out_not_result
                   , std::vector<int8_t> & ring_no 
                   , float height  
                   , float angle_min 
                   , float angle_max 
                  )
{
	//globaldebugdata::BasicFilter tg_debug_test_0 (GlobalDebugData::getDebugTest0Flag());
	
	ROS_ASSERT(angle_max >= angle_min);
	
	//ROS_ASSERT((true) && (_ring_no < 16));
	BOOST_FOREACH(int8_t _ring_no, ring_no)	
	    ROS_ASSERT((0 <= _ring_no) && (_ring_no < 16));
	
	//ROS_INFO_STREAM_FILTER(globaldebugdata::DebugFlags::getFlag0(), "Streaming messages!!!");
	//ROS_WARN_STREAM_FILTER_NAMED(&tg_debug_test_0, "kentut", "Streaming messages!!!");
	//ROS_WARN_STREAM_FILTER_NAMED(globaldebugdata::DebugFlags::getFlag0(), "kentut", "Streaming messages!!!");
	
	
	// this is needed coz outMsg_debug & outMsg_high aren't
    // exactly produced from cloud_transformed (i.e. need init)
	clearFrame(cloud_out_result);
	clearFrame(cloud_out_not_result);
	
	bool b_search_for_height = false;
	if (height > 0) b_search_for_height = true;

    bool b_search_for_one_ring = false;
    //if (16 > ring_no && ring_no >=3) b_search_for_one_ring = true;
    
    bool b_search_for_angle = false;
    if (angle_min !=0 && angle_max !=0) bool b_search_for_angle = true;
	
	int _point_counter = 0;
	min_angle_ = angle_min;
	min_index_ = 0;
	
	bool _point_in_result = false;
	

	BOOST_FOREACH(VPoint _point, *cloud_in){
		_point_in_result = false;
		//if (!b_search_for_one_ring || _point.ring == ring_no)
		int8_t _iring_no_test[] = {_point.ring, _point.ring};
		std::vector<int8_t> _ring_no_test (_iring_no_test , _iring_no_test+sizeof(_iring_no_test)/sizeof(_iring_no_test[0]));
		if (std::find(ring_no.begin(), ring_no.end(), _point.ring) != ring_no.end())
		{
			if (!b_search_for_height || _point.z <= height)
			{
				float _angle  = atan2f(_point.y,_point.x)*180/PI;
				if (!b_search_for_angle || 
					(angle_min < _angle && _angle < angle_max)
				   )
				{
					cloud_out_result->push_back(_point);

				    angle_data_[_point_counter] = _angle;
					
					if (_angle > min_angle_) {
						min_angle_ = _angle;
						min_index_ = _point_counter;
					}
					
					_point_in_result = true;
					_point_counter++;
				}
			}
		}
		if (!_point_in_result)
		{
					cloud_out_not_result->push_back(_point);
		}
	}
	
}

}
