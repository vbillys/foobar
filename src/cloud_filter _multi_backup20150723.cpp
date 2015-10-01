/*
 * Known issues:
 *  - at end problem (SOLVED)
 *  - noises, z constraint may be? -> can use prediction kah?
 *  - -> may be try to use multiple tau values... or RAN SAC
 *  - obstacle detection not implemented
 *  - prediction on y or angle constraint, may yet best accumulate past
 *  - left and right curve modeling?
 * 
 * */


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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <stdio.h>

#include <hiredis.h>
//#define WITH_REDIS
#include "debug_print.cpp"

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub, pub_debug, pub_curb, pub_transformed;

FILE *fp;
unsigned int  g_info_counter = 0;
float g_angle_data[32000];
float g_angle_data_ring[16][1000];
float g_angle_data_ring_org[16][1000];

int g_flag_process_ring = 16;
double g_flag_simul_until = 0;

VPointCloud::Ptr g_filtered_points(new VPointCloud());
VPointCloud::Ptr g_outMsg_debug(new VPointCloud());

void cloud_cb (const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
	//ROS_INFO_STREAM("Yo, whats up I got data!");
	VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = "filtered_velodyne";
    pub.publish(outMsg);
    
}

#define PI 3.14159265

typedef struct {
	float left_x;
	float right_x;
	float left_y;
	float right_y;
	bool  got_left;
	bool  got_right;
	bool  data_invalid;
}CurbRing;

CurbRing cloudProcess ( const VPointCloud::Ptr& outMsg_debug , int ring_no);

void cloud_cb_3 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	
	VPointCloud::Ptr cloud(new VPointCloud());
	VPointCloud::Ptr cloud_transformed(new VPointCloud());
	pcl::fromROSMsg(*cloud_msg , *cloud);
	
    // Define Eigen matrix(transformaton matrix) rotation about z-axis
    double theta  = 22 * PI/180;
    double thetaX = -2 * PI/180;
    
    // Method II for (transformaton matrix) Affine transformation
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  
    // Define a translation of 2.5 meters on the z axis.
    transform_2.translation() << 0.0, 0.0, 1.5;
  
    // The same rotation matrix as before; tetha radians arround Y axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
  
    // The same rotation matrix as before; tetha radians arround X axis
    transform_2.rotate (Eigen::AngleAxisf (thetaX, Eigen::Vector3f::UnitX()));
  
    // perform transformation using 
    pcl::transformPointCloud(*cloud,*cloud_transformed,transform_2);

#define APPROX_HALF_ANGLE 5
#define LIMIT_ANGLE 90//175//90
#define Z_CUT_LIMIT 0.5

    VPointCloud::Ptr outMsg_high(new VPointCloud());
    outMsg_high->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_high->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg_high->height = 1;
    
    VPointCloud::Ptr outMsg_debug(new VPointCloud());
    outMsg_debug->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_debug->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg_debug->height = 1;
    
    int _point_counter = 0;

    for (int next = 0; next < cloud_transformed->points.size(); ++next)
    {
		velodyne_pointcloud::PointXYZIR _point = cloud_transformed->at(next);
		float _angle  = atan2f(_point.y,_point.x)*180/PI;
		
		if ( _point.z <= Z_CUT_LIMIT
		   //&& _point.ring != RING_NO 
		   && -LIMIT_ANGLE < _angle && _angle < LIMIT_ANGLE
		   ){
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			outMsg_debug->push_back(_point_new);
			g_angle_data[_point_counter] = _angle;
			_point_counter++;
		}
		else
		{
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			//outMsg_debug->push_back(_point_new);
			outMsg_high->push_back(_point_new);
		}
	}
    
    VPointCloud::Ptr outMsg_curb(new VPointCloud());
    outMsg_curb->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_curb->header.frame_id = cloud->header.frame_id;
    outMsg_curb->height = 1;
    
    g_filtered_points->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    g_filtered_points->header.frame_id = cloud->header.frame_id;
    g_filtered_points->clear();
    g_filtered_points->height = 1;
    
    g_outMsg_debug->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    g_outMsg_debug->header.frame_id = cloud->header.frame_id;
    g_outMsg_debug->clear();
    g_outMsg_debug->height = 1;
    
    unsigned int ring_nos[16] = {3,4,5,6,7,8,9,10,11,12,13,14,15,17,17,17};
    //int _ring_no_being_processed = 6;
    if (16 > g_flag_process_ring && g_flag_process_ring >=3)
    {
		ring_nos[0] = g_flag_process_ring;
		ring_nos[1] = 17;
	}
	else
	{
		
	}
    
    for (int _ring_no_index = 0; 
         ring_nos[_ring_no_index] < 16 && _ring_no_index < 16; 
         _ring_no_index++
        )
    {
	  unsigned int _ring_no_being_processed = ring_nos[_ring_no_index];
      CurbRing _result = cloudProcess ( outMsg_debug , _ring_no_being_processed );
      if (!_result.data_invalid)
      {
		  
		  if (_result.got_left)
		  {
			  velodyne_pointcloud::PointXYZIR _point_new;
			  _point_new.x = _result.left_x;
			  _point_new.y = _result.left_y;
			  _point_new.z = 0;
			  _point_new.ring = 0;
			  _point_new.intensity = 0;
			  outMsg_curb->push_back(_point_new);
			  fprintf(fp,"%d\t%d\t%.3f\t%.3f\n"
		        ,g_info_counter
		        ,_ring_no_being_processed
		        ,_result.left_x
		        ,_result.left_y
		       );
		  }
		  else
		  {
			  debug_print_0("no left  curb at ring %d ...\n", _ring_no_being_processed);
		  }
		  if (_result.got_right)
		  {
			  velodyne_pointcloud::PointXYZIR _point_new;
			  _point_new.x = _result.right_x;
			  _point_new.y = _result.right_y;
			  _point_new.z = 0;
			  _point_new.ring = 0;
			  _point_new.intensity = 0;
			  outMsg_curb->push_back(_point_new);
		  }
		  else
		  {
			  debug_print_0("no right curb at ring %d ...\n", _ring_no_being_processed);
		  }
      }
      else
      {
		  debug_print_0("data invalid at ring %d ...\n", _ring_no_being_processed);
	  }
    }
    ++g_info_counter;
    pub_debug.publish(g_outMsg_debug);
    pub_transformed.publish(outMsg_high);
    pub_curb.publish(outMsg_curb);
    pub.publish(g_filtered_points);
    
    debug_print_0("--- 0 ---\n");
    debug_print_1("--- 1 ---\n");
    debug_print_2("--- 2 ---\n");
    debug_print_3("--- 3 ---\n");
}

CurbRing cloudProcess ( const VPointCloud::Ptr& outMsg_debug , int ring_no)
{
	//pub_debug.publish(outMsg_debug);
	CurbRing result;
	result.data_invalid=false;
	result.got_left=false;
	result.got_right=false;
	
	VPointCloud::Ptr outMsg(new VPointCloud());
	outMsg->header.stamp = outMsg_debug->header.stamp;
    outMsg->header.frame_id = outMsg_debug->header.frame_id;//"filtered_velodyne";
    outMsg->height = 1;
    
    bool _first = true;
    float first_angle, last_angle, prev_angle;
    float min_angle = -90;
    size_t min_index, cur_index, half_index; cur_index = 0;
    bool _found_half = false;
    
    for (int next = 0; next < outMsg_debug->points.size(); ++next)
    {
		velodyne_pointcloud::PointXYZIR _point = outMsg_debug->at(next);
		if ( _point.ring == ring_no )
		{
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			outMsg->push_back(_point_new);
			g_angle_data_ring[ring_no][cur_index] = g_angle_data[next];
			if (min_angle < g_angle_data[next]) 
			{
			  min_angle = g_angle_data[next];
			  min_index = cur_index;
			}
			
			cur_index++;
		}
	}
    
    if (outMsg->points.size() <=30) {
		char ts[128];
		sprintf(ts, "no or too few points got..ring %d!!", ring_no);
		ROS_INFO_STREAM(ts);
		debug_print_1("INVALID too few points ..ring %d!!", ring_no);
		result.data_invalid=true;
		return result;
	}
	
	VPointCloud::Ptr cloud_org(new VPointCloud());
	cloud_org->header.stamp    = outMsg_debug->header.stamp;
    cloud_org->header.frame_id = outMsg_debug->header.frame_id;//"filtered_velodyne";
    cloud_org->height = 1;
    
    for (int next = 0; next < outMsg->points.size(); next++)
    {
		int _index_reindex = next + min_index;
		if (_index_reindex >= outMsg->points.size()) _index_reindex -= outMsg->points.size();
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = outMsg->points[_index_reindex].ring;
		_point_new.x    = outMsg->points[_index_reindex].x;
		_point_new.y    = outMsg->points[_index_reindex].y;
		_point_new.z    = outMsg->points[_index_reindex].z;
		_point_new.intensity = outMsg->points[_index_reindex].intensity;
		//_point_new.intensity = next;
		cloud_org->push_back(_point_new);
		_point_new.intensity = next;
		//g_filtered_points->push_back(_point_new);
		g_outMsg_debug->push_back(_point_new);
		g_angle_data_ring_org [ring_no][next] = g_angle_data_ring[ring_no][_index_reindex];
		float _angle  = g_angle_data_ring[ring_no][_index_reindex];
		if ( (!_found_half) && (-APPROX_HALF_ANGLE < _angle && _angle < APPROX_HALF_ANGLE) ){
			    _found_half = true;
			    half_index  = next;
		}
	}
	
	// filter bug on the velodyne firmware may be...
	float _first_cloud_org_angle_degree = atan2f(cloud_org->points[0].y,cloud_org->points[0].x)*180/3.1459;
	float  _last_cloud_org_angle_degree = atan2f(cloud_org->points[cloud_org->points.size()-1].y, cloud_org->points[cloud_org->points.size()-1].x)*180/3.1459;
	if (_first_cloud_org_angle_degree - _last_cloud_org_angle_degree < 20.) {
		debug_print_1("%s","NOT PUBLISHING!!! \n"); 
		result.data_invalid=true;
		return result;
	}
	else
	{
		debug_print_1("PUBLISHING!!! %d %d\n,", cloud_org->points.size(), half_index); 
	}
	
	if (!_found_half){
		half_index = cloud_org->points.size()/2; 
	    ROS_INFO_STREAM("WARN:half point not found!..using div..at " << ring_no);
	}
	
	//if (ring_no == 7) pub.publish(cloud_org);
	
#define CROSS_PROD_DIST 15//15
#define CURB_ZCROSS_THRESHOLD 0.02//0.005 //0.02
	
	int t_bef, t_after;
	int tau = CROSS_PROD_DIST;//15; //  15  5
	float filter_data[cloud_org->points.size()];
	
	debug_print_3("\n");
	for (int j = 0; j < cloud_org->points.size(); j++)
    {
		t_bef   = j - tau; if (t_bef < 0) t_bef = 0;
		t_after = j + tau; if (t_after > cloud_org->points.size()-1) t_after = cloud_org->points.size()-1;
		//filter_data[j]   = //2*radial_data[j]
		                   //+radial_data[t_after]
		                   //radial_data[j]-radial_data[t_bef]
		                   //-fabs(radial_data[t_bef]-radial_data[t_after])
		                   //fabs(radial_data[t_bef]-radial_data[t_after])
		//                  ;
		
		float _xvec1 =  cloud_org->points[j].x - cloud_org->points[t_bef].x;
		float _yvec1 =  cloud_org->points[j].y - cloud_org->points[t_bef].y;
		//float _zvec1 =  cloud_org->points[j].z - cloud_org->points[t_bef].z;
		
		float _xvec2 = -cloud_org->points[j].x + cloud_org->points[t_after].x;
		float _yvec2 = -cloud_org->points[j].y + cloud_org->points[t_after].y;
		//float _zvec2 = -cloud_org->points[j].z + cloud_org->points[t_after].z;
		
		//float _cross_i = _yvec1*_zvec2 - _zvec1*_yvec2;
		//float _cross_j = _zvec1*_xvec2 - _xvec1*_zvec2;
		float _cross_k = _xvec1*_yvec2 - _yvec1*_xvec2;
		
		//filter_data[j]     = sqrtf(_cross_i*_cross_i + _cross_j*_cross_j + _cross_k*_cross_k);
		//filter_data[j]     =  fabs(_cross_k);
		filter_data[j]     =  (_cross_k);
		
		//cloud_org->points[j].intensity = filter_data[j];
		//cloud_org->points[j].intensity = _cross_k;
		//cloud_org->points[j].intensity = fabs(_cross_k);
		
		if (filter_data[j] > CURB_ZCROSS_THRESHOLD)
		{
			cloud_org->points[j].intensity =  1.;
			debug_print_3("%d : %.3f %.3f %.3f %.3f \n", j, filter_data[j], cloud_org->points[j].z, cloud_org->points[j].y, g_angle_data_ring_org [ring_no][j]);
			
		}
		else if (filter_data[j] < -CURB_ZCROSS_THRESHOLD)
		{
			cloud_org->points[j].intensity = -1.;
			debug_print_3("%d : %.3f %.3f %.3f %.3f \n", j, filter_data[j], cloud_org->points[j].z, cloud_org->points[j].y, g_angle_data_ring_org [ring_no][j]);
		}
		else
		{
			cloud_org->points[j].intensity = 0.;
		}
		//printf("%.9f ",cloud_org->points[j].intensity);
		//printf("%.3f ",filter_data[j]);
		g_filtered_points->push_back(cloud_org->points[j]);
	}
	debug_print_3("\n");
	
#define CURB_ZHEIGHT_THRESHOLD 0.1 //0.05
#define CURB_YDIFF_THRESHOLD 2.//1.5//0.5
	
	//VPointCloud::Ptr outMsg_curb(new VPointCloud());
    //outMsg_curb->header.stamp = outMsg_debug->header.stamp;
    //outMsg_curb->header.frame_id = outMsg_debug->header.frame_id;//"filtered_velodyne";
    //outMsg_curb->height = 1;
	
	bool _found_negative = false;
	bool _found_positive = false;
	uint8_t _follow_positive = 0;
	uint8_t _follow_negative = 0;
	uint8_t _last_follow_positive = 0;
	uint8_t _last_follow_negative = 0;
	float _min_negative_crossprod, _last_min_negative_crossprod;
	float _max_positive_crossprod, _last_max_positive_crossprod;
	int _min_negative_index, _last_min_negative_index;
	int _max_positive_index, _last_max_positive_index;
	int _min_x_index, _last_min_x_index;
	int _max_x_index, _last_max_x_index;
	float _min_x, _last_min_x;
	float _max_x, _last_max_x;
	int _curb_point_index;
	bool _search_next = false;
	bool _curb_found  = false;
	for (int j = half_index; j < cloud_org->points.size(); j++)
    {
		
		if (filter_data[j] < 0)
		{
			if(!_found_negative){
			  _last_follow_positive = _follow_positive;
			  _found_negative = true;
			  _found_positive = false;
			  _last_min_negative_index     = _min_negative_index;
			  _last_min_negative_crossprod = _min_negative_crossprod;
			  _min_negative_crossprod =  filter_data[j];
			  _min_negative_index = j;
			  
			  //_last_min_x = _min_x;
			  _last_max_x = _max_x;
			  //_last_min_x_index = _min_x_index;
			  _last_max_x_index = _max_x_index;
			  //_min_x = cloud_org->points[j].x;
			  _max_x = cloud_org->points[j].x;
			  //_min_x_index = j;
			  _max_x_index = j;
			  debug_print_5("right search found negative %d %d\n", j, _last_follow_positive);
		    }
			_follow_negative++;
			_follow_positive = 0;
			if (filter_data[j] < _min_negative_crossprod) {
			    _min_negative_crossprod = filter_data[j];
			    _min_negative_index = j;
			    //printf("min negative index found: %d...\n",j);
			}
			
			//if (cloud_org->points[j].x < _min_x) {
				//_min_x = cloud_org->points[j].x;
				//_min_x_index = j;
			//}
			if (cloud_org->points[j].x > _max_x) {
				_max_x = cloud_org->points[j].x;
				_max_x_index = j;
			}
			
			if (_last_follow_negative > 6 && _last_follow_positive > 6 && !_search_next)
			{
				bool _match_curb = false;
				if (//cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y < CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y > -CURB_YDIFF_THRESHOLD
				    cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD
				    && cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y < CURB_YDIFF_THRESHOLD
				    && cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y > -CURB_YDIFF_THRESHOLD
				   )
				{
					//velodyne_pointcloud::PointXYZIR _point_new;
					//_point_new.ring = cloud_org->points[_curb_point_index].ring;
					//_point_new.x    = cloud_org->points[_curb_point_index].x;
					//_point_new.y    = cloud_org->points[_curb_point_index].y;
					//_point_new.z    = cloud_org->points[_curb_point_index].z;
					//_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
					//outMsg_curb->push_back(_point_new);
					debug_print_2("MATCH CURB!\n");
					_match_curb = true;
			    }
			    else
			    {
					debug_print_2("NO MATCH CURB!\n");
					_search_next = true;
					//continue;
				}
					//int _curb_point_index = _last_min_negative_index+(_max_positive_index - _last_min_negative_index)/2;
					debug_print_2("found right!!... at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
					
					
			    //else
			    //{
					debug_print_2("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
				//}
				//printf("GIVE UP CURB RIGHT!\n");
				if (_match_curb){
				  _curb_found = true;
				  break;
			    }
			}
		}
		else
		{
			if(!_found_positive){
			  _last_follow_negative = _follow_negative;
			  _found_negative = false;
			  _found_positive = true;
			  _last_max_positive_index     = _max_positive_index;
			  _last_max_positive_crossprod = _max_positive_crossprod;
			  _max_positive_crossprod = filter_data[j];
			  _max_positive_index = j;
			  if (_last_follow_negative > 6){
				_curb_point_index = j;
			  }
			  _search_next = false;
			  
			  _last_min_x = _min_x;
			  //_last_max_x = _max_x;
			  _last_min_x_index = _min_x_index;
			  //_last_max_x_index = _max_x_index;
			  _min_x = cloud_org->points[j].x;
			  //_max_x = cloud_org->points[j].x;
			  _min_x_index = j;
			  //_max_x_index = j;
			  debug_print_5("right search found positive %d %d\n", j, _last_follow_negative);
		    }
			_follow_positive++;
			_follow_negative = 0;
			if (filter_data[j] > _max_positive_crossprod) {
			    _max_positive_crossprod = filter_data[j];
			    _max_positive_index = j;
			}
			
			if (cloud_org->points[j].x < _min_x) {
				_min_x = cloud_org->points[j].x;
				_min_x_index = j;
			}
			//if (cloud_org->points[j].x > _max_x) {
				//_max_x = cloud_org->points[j].x;
				//_max_x_index = j;
			//}
			    
			    
			//if (_last_follow_negative > 6 && _follow_positive > 6)
			//{
				//printf("found right!!... at %d : %.3f %.3f %.3f %.3f %d : %.3f %.3f %.3f %.3f \n"
				       //, j
				       //, filter_data[j]
				       //, cloud_org->points[j].z
				       //, cloud_org->points[j].x
				       //, cloud_org->points[j].y
				       
				       //, _min_negative_index
				       //, _min_negative_crossprod
				       //, cloud_org->points[_min_negative_index].z
				       //, cloud_org->points[_min_negative_index].x
				       //, cloud_org->points[_min_negative_index].y
				      //);
				//break;
			//}
			
		}
        
	}
	// TO PREVENT MISS AT END
	if (_last_follow_negative > 6 && _follow_positive > 6)
	{
		debug_print_2("MAy MISS at END!\n");
		if (cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD)
		{
			debug_print_2("MATCH CURB!\n");
			//velodyne_pointcloud::PointXYZIR _point_new;
			//_point_new.ring = cloud_org->points[_curb_point_index].ring;
			//_point_new.x    = cloud_org->points[_curb_point_index].x;
			//_point_new.y    = cloud_org->points[_curb_point_index].y;
			//_point_new.z    = cloud_org->points[_curb_point_index].z;
			//_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
			//outMsg_curb->push_back(_point_new);
			_curb_found = true;
		}
		else
		{
			debug_print_2("NO MATCH CURB!\n");
		}
		debug_print_2("found right!!... at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
		debug_print_2("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
	}
	
	if (!_curb_found) {
		debug_print_2("GIVE UP CURB RIGHT!\n");
	}
	else
	{
		result.got_right = true;
		result.right_x = cloud_org->points[_curb_point_index].x;
		result.right_y = cloud_org->points[_curb_point_index].y;
	}
		         
		         
	
	
	
	_found_negative = false;
	_found_positive = false;
	_follow_positive = 0;
	_follow_negative = 0;
	_last_follow_positive = 0;
	_last_follow_negative = 0;
	//_min_negative_crossprod =  0.;
	//_max_positive_crossprod = -1.;
	_curb_found = false;
	_search_next = false;
	int _jj ;
	for (int j = half_index; j >= 0; j--)
    {
		_jj = j;
		if (filter_data[j] < 0)
		{
			if(!_found_negative){
			  _last_follow_positive = _follow_positive;
			  _found_negative = true;
			  _found_positive = false;
			  _last_min_negative_index     = _min_negative_index;
			  _last_min_negative_crossprod = _min_negative_crossprod;
			  _min_negative_crossprod =  filter_data[j];
			  _min_negative_index = j;
			  
			  //_last_min_x = _min_x;
			  _last_max_x = _max_x;
			  //_last_min_x_index = _min_x_index;
			  _last_max_x_index = _max_x_index;
			  //_min_x = cloud_org->points[j].x;
			  _max_x = cloud_org->points[j].x;
			  //_min_x_index = j;
			  _max_x_index = j;
			  debug_print_4("left search found negative %d %d\n", j, _last_follow_positive);
		    }
			_follow_negative++;
			_follow_positive = 0;
			if (filter_data[j] < _min_negative_crossprod) {
			    _min_negative_crossprod = filter_data[j];
			    _min_negative_index = j;
			}
			
			//if (cloud_org->points[j].x < _min_x) {
				//_min_x = cloud_org->points[j].x;
				//_min_x_index = j;
			//}
			if (cloud_org->points[j].x > _max_x) {
				_max_x = cloud_org->points[j].x;
				_max_x_index = j;
			}
			
			if (_last_follow_negative > 6 && _last_follow_positive > 6 && !_search_next)
			{
				bool _match_curb = false;
				if (//cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y > -CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y < CURB_YDIFF_THRESHOLD
				    cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD
				    && cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y < CURB_YDIFF_THRESHOLD
				    && cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y > -CURB_YDIFF_THRESHOLD
				    
				   )
				{
					//velodyne_pointcloud::PointXYZIR _point_new;
					//_point_new.ring = cloud_org->points[_curb_point_index].ring;
					//_point_new.x    = cloud_org->points[_curb_point_index].x;
					//_point_new.y    = cloud_org->points[_curb_point_index].y;
					//_point_new.z    = cloud_org->points[_curb_point_index].z;
					//_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
					//outMsg_curb->push_back(_point_new);
					debug_print_2("MATCH CURB!\n");
					_match_curb = true;
			    }
			    else
			    {
					debug_print_2("NO MATCH CURB!\n");
					_search_next = true;
					//continue;
				}
					//int _curb_point_index = _last_min_negative_index+(_max_positive_index - _last_min_negative_index)/2;
					debug_print_2("found left!!...  at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
					
					
			    //else
			    //{
					debug_print_2("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
				//}
				//printf("GIVE UP CURB LEFT!\n");
				if (_match_curb){
				  _curb_found = true;
				  break;
			    }
			}
		}
		else
		{
			if(!_found_positive){
			  _last_follow_negative = _follow_negative;
			  _found_negative = false;
			  _found_positive = true;
			  _last_max_positive_index     = _max_positive_index;
			  _last_max_positive_crossprod = _max_positive_crossprod;
			  _max_positive_crossprod = filter_data[j];
			  _max_positive_index = j;
			  if (_last_follow_negative > 6){
				_curb_point_index = j;
			  }
			  _search_next = false;
			  
			  _last_min_x = _min_x;
			  //_last_max_x = _max_x;
			  _last_min_x_index = _min_x_index;
			  //_last_max_x_index = _max_x_index;
			  _min_x = cloud_org->points[j].x;
			  //_max_x = cloud_org->points[j].x;
			  _min_x_index = j;
			  //_max_x_index = j;
			  debug_print_4("left search found positive %d %d\n", j, _last_follow_negative);
		    }
			_follow_positive++;
			_follow_negative = 0;
			if (filter_data[j] > _max_positive_crossprod) {
			    _max_positive_crossprod = filter_data[j];
			    _max_positive_index = j;
			}
			
			if (cloud_org->points[j].x < _min_x) {
				_min_x = cloud_org->points[j].x;
				_min_x_index = j;
			}
			//if (cloud_org->points[j].x > _max_x) {
				//_max_x = cloud_org->points[j].x;
				//_max_x_index = j;
			//}
			
			//if (_last_follow_negative > 6 && _follow_positive > 6)
			//{
				//printf("found left!! ... at %d : %.3f %.3f %.3f %.3f %d : %.3f %.3f %.3f %.3f \n"
				       //, j
				       //, filter_data[j]
				       //, cloud_org->points[j].z
				       //, cloud_org->points[j].x
				       //, cloud_org->points[j].y
				       
				       //, _min_negative_index
				       //, _min_negative_crossprod
				       //, cloud_org->points[_min_negative_index].z
				       //, cloud_org->points[_min_negative_index].x
				       //, cloud_org->points[_min_negative_index].y
				      //);
				//break;
			//}
			
		}

	}
	if (_last_follow_negative > 6 && _follow_positive > 6 )//&& j == cloud_org->points.size() - 1)
    {
		debug_print_2("MAy MISS at END! j=%d size=%d\n", _jj , cloud_org->points.size());
		if (cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD)
		{
			//velodyne_pointcloud::PointXYZIR _point_new;
			//_point_new.ring = cloud_org->points[_curb_point_index].ring;
			//_point_new.x    = cloud_org->points[_curb_point_index].x;
			//_point_new.y    = cloud_org->points[_curb_point_index].y;
			//_point_new.z    = cloud_org->points[_curb_point_index].z;
			//_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
			//outMsg_curb->push_back(_point_new);
			debug_print_2("MATCH CURB!\n");
			_curb_found = true;
		}
		else
		{
			debug_print_2("NO MATCH CURB!\n");
		}
		debug_print_2("found left!!...  at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
					   , _max_positive_index
					   , _max_positive_crossprod
					   , cloud_org->points[_min_x_index].z // _max_positive_index
					   , cloud_org->points[_max_positive_index].x
					   , cloud_org->points[_min_x_index].y // _max_positive_index
					   
					   , _last_min_negative_index
					   , _last_min_negative_crossprod
					   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
					   , cloud_org->points[_last_min_negative_index].x
					   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
					   
					   , _curb_point_index
					   , filter_data[_curb_point_index]
					   , cloud_org->points[_curb_point_index].z
					   , cloud_org->points[_curb_point_index].x
					   , cloud_org->points[_curb_point_index].y
					   
					   , _min_x_index
					   , _last_max_x_index
					  );
					  
		debug_print_2("criterion:  %.3f %.3f\n"
					   //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					   //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					   , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					   , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					  );
		
	}
	if (!_curb_found) {
		debug_print_2("GIVE UP CURB LEFT!\n");
	}
	else
	{
		result.got_left = true;
		result.left_x = cloud_org->points[_curb_point_index].x;
		result.left_y = cloud_org->points[_curb_point_index].y;
	}
	
	return result;
}

void cloud_cb_2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    VPointCloud::Ptr cloud(new VPointCloud());
    VPointCloud::Ptr cloud_org(new VPointCloud());
    VPointCloud::Ptr outMsg(new VPointCloud());
    VPointCloud::Ptr cloud_transformed(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud);
    
    //std::vector<VPointCloud::Ptr> outMsgs;
    
    //VPointCloud::Ptr *outMsgs;
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr **cloud_clusters_ptr; 

    //for (int i=0; i<16; i++) { 
    //  cloud_clusters_ptr[i] = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
    //} 
    
    //VPointCloud::Ptr *outMsgs = new VPointCloud::Ptr (new VPointCloud)[16];
    //for(int j=0;j<16;j++) {
	//	//outMsgs[j] = new VPointCloud();
	//	outMsgs.push_back(new VPointCloud::Ptr (new VPointCloud()));
	//}
	//pcl::fromROSMsg(*cloud_msg , *outMsgs[0]);
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud_clusters = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>) [numClusters]; 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_pcl (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg(*cloud_msg , *cloud_pcl);
    
    outMsg->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg->height = 1;
    
    VPointCloud::Ptr outMsg_debug(new VPointCloud());
    outMsg_debug->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_debug->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg_debug->height = 1;
    
    // Define Eigen matrix(transformaton matrix) rotation about z-axis
    double theta  = 22 * PI/180;
    double thetaX = -2 * PI/180;
    
    // Method II for (transformaton matrix) Affine transformation
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  
    // Define a translation of 2.5 meters on the z axis.
    transform_2.translation() << 0.0, 0.0, 1.5;
  
    // The same rotation matrix as before; tetha radians arround Y axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
  
    // The same rotation matrix as before; tetha radians arround X axis
    transform_2.rotate (Eigen::AngleAxisf (thetaX, Eigen::Vector3f::UnitX()));
  
    // perform transformation using 
    pcl::transformPointCloud(*cloud,*cloud_transformed,transform_2);
    //pcl::transformPointCloud(*cloud_pcl,*cloud_transformed_pcl,transform_2);
    
    // Create the filtering object
    //pcl::PassThrough<velodyne_pointcloud::PointXYZIR> pass;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_transformed_pcl);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.25, 0.5);//2
    //pass.setFilterLimitsNegative (true);
    //pass.filter (*cloud_transformed_pcl);
    
    bool _first = true;
    float first_angle, last_angle, prev_angle;
    float min_angle = -90;
    size_t min_index, cur_index, half_index; cur_index = 0;
    bool _found_half = false;


#define RING_NO 7//2//10
     
    VPointCloud::Ptr outMsg_high(new VPointCloud());
    outMsg_high->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_high->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg_high->height = 1;
    //printf("\n");
    for (size_t next = 0; next < cloud_transformed->points.size(); ++next)
    {//printf ("%d ",cloud_transformed->points[next].ring);
		velodyne_pointcloud::PointXYZIR _point = cloud_transformed->at(next);
		float _angle  = atan2f(_point.y,_point.x)*180/3.1459;
		if ( _point.z <= Z_CUT_LIMIT
		   && _point.ring == RING_NO 
		   && -LIMIT_ANGLE < _angle && _angle < LIMIT_ANGLE
		   ){
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			outMsg->push_back(_point_new);
			if (min_angle < _angle) 
			{
			  min_angle = _angle;
			  min_index = cur_index;
			}
			
			cur_index++;
		}
		else
		{
			
		}
		
		// DEBUG TSART
		if ( _point.z <= Z_CUT_LIMIT
		   && _point.ring != RING_NO 
		   && -LIMIT_ANGLE < _angle && _angle < LIMIT_ANGLE
		   ){
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			outMsg_debug->push_back(_point_new);
			
		}
		else
		{
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			//outMsg_debug->push_back(_point_new);
			outMsg_high->push_back(_point_new);
		}
		// DEBUG END
		
		if (_first){
		      _first = false;
		      first_angle = _angle;
		      
		}
		
		last_angle = _angle;
	}
	//printf("\n");
	//DEBUG
	//pub_debug.publish(outMsg_debug);
	
	
	cloud_org->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    cloud_org->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    cloud_org->height = 1;
    _first = true;
    
    float angle_data[outMsg->points.size()];
	
	for (size_t next = 0; next < outMsg->points.size(); next++)
    {
		size_t _index_reindex = next + min_index;
		if (_index_reindex >= outMsg->points.size()) _index_reindex -= outMsg->points.size();
		//if (next < 450) outMsg->points[_index_reindex].intensity = 25;
		//else outMsg->points[_index_reindex].intensity = 100;
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = outMsg->points[_index_reindex].ring;
		_point_new.x    = outMsg->points[_index_reindex].x;
		_point_new.y    = outMsg->points[_index_reindex].y;
		_point_new.z    = outMsg->points[_index_reindex].z;
		_point_new.intensity = outMsg->points[_index_reindex].intensity;
		cloud_org->push_back(_point_new);
		
		float _angle  = atan2f(_point_new.y,_point_new.x)*180/3.1459;
		angle_data[next]  = _angle;
		
		if ( (!_found_half) && (-APPROX_HALF_ANGLE < _angle && _angle < APPROX_HALF_ANGLE) ){
			    _found_half = true;
			    half_index = next;
		}
		
		if (_first){
		      _first = false;
		      
		      
		}
		else
		{
		    if (false)//((_angle - prev_angle) < 0)
			{
				//_invalid_noisy_data = true;
				return;
		    }
		}
		prev_angle = _angle;
	}
    
    if (outMsg->points.size() <=30) {ROS_INFO_STREAM("no or too few points got..!!");return;}
    
    // filter bug on the velodyne firmware may be...
	float _first_cloud_org_angle_degree = atan2f(cloud_org->points[0].y,cloud_org->points[0].x)*180/3.1459;
	float _last_cloud_org_angle_degree  = atan2f(cloud_org->points[cloud_org->points.size()-1].y, cloud_org->points[cloud_org->points.size()-1].x)*180/3.1459;
	if (_first_cloud_org_angle_degree - _last_cloud_org_angle_degree < 20.) {
		printf("NOT PUBLISHING!!! \n"); 
		return;
	}
	else
	{
		printf("PUBLISHING!!! %d %d\n,", cloud_org->points.size(), half_index); 
	}
	
	if (!_found_half){
		half_index = cloud_org->points.size()/2; 
	    ROS_INFO_STREAM("WARN:half point not found!..using div");
	}
	
//#define CROSS_PROD_DIST 15//15
//#define CURB_ZCROSS_THRESHOLD 0.005 //0.02
	
	int t_bef, t_after;
	int tau = CROSS_PROD_DIST;//15; //  15  5
	int tau2 = 10;
	//int tau3 = 20;
	float filter_data[cloud_org->points.size()];
	
	printf("\n");
	for (int j = 0; j < cloud_org->points.size(); j++)
    {
		t_bef   = j - tau; if (t_bef < 0) t_bef = 0;
		t_after = j + tau; if (t_after > cloud_org->points.size()-1) t_after = cloud_org->points.size()-1;
		//filter_data[j]   = //2*radial_data[j]
		                   //+radial_data[t_after]
		                   //radial_data[j]-radial_data[t_bef]
		                   //-fabs(radial_data[t_bef]-radial_data[t_after])
		                   //fabs(radial_data[t_bef]-radial_data[t_after])
		//                  ;
		
		float _xvec1 =  cloud_org->points[j].x - cloud_org->points[t_bef].x;
		float _yvec1 =  cloud_org->points[j].y - cloud_org->points[t_bef].y;
		//float _zvec1 =  cloud_org->points[j].z - cloud_org->points[t_bef].z;
		
		float _xvec2 = -cloud_org->points[j].x + cloud_org->points[t_after].x;
		float _yvec2 = -cloud_org->points[j].y + cloud_org->points[t_after].y;
		//float _zvec2 = -cloud_org->points[j].z + cloud_org->points[t_after].z;
		
		//float _cross_i = _yvec1*_zvec2 - _zvec1*_yvec2;
		//float _cross_j = _zvec1*_xvec2 - _xvec1*_zvec2;
		float _cross_k = _xvec1*_yvec2 - _yvec1*_xvec2;
		
		//filter_data[j]     = sqrtf(_cross_i*_cross_i + _cross_j*_cross_j + _cross_k*_cross_k);
		//filter_data[j]     =  fabs(_cross_k);
		filter_data[j]     =  (_cross_k) ;// fabs(angle_data[t_after] - angle_data[t_bef]);
		
		//t_bef   = j - tau2; if (t_bef < 0) t_bef = 0;
		//t_after = j + tau2; if (t_after > cloud_org->points.size()-1) t_after = cloud_org->points.size()-1;
		//_xvec1 =  cloud_org->points[j].x - cloud_org->points[t_bef].x;
		//_yvec1 =  cloud_org->points[j].y - cloud_org->points[t_bef].y;
		//_xvec2 = -cloud_org->points[j].x + cloud_org->points[t_after].x;
		//_yvec2 = -cloud_org->points[j].y + cloud_org->points[t_after].y;
		//_cross_k = _xvec1*_yvec2 - _yvec1*_xvec2;
		//filter_data[j]     +=  (_cross_k);
		
		//cloud_org->points[j].intensity = filter_data[j];
		//cloud_org->points[j].intensity = _cross_k;
		//cloud_org->points[j].intensity = fabs(_cross_k);
		
		if (filter_data[j] > CURB_ZCROSS_THRESHOLD)
		{
			cloud_org->points[j].intensity =  1.;
			//printf("%d : %.3f %.3f %.3f \n", j, filter_data[j], cloud_org->points[j].z, cloud_org->points[j].y);
			
		}
		else if (filter_data[j] < -CURB_ZCROSS_THRESHOLD)
		{
			cloud_org->points[j].intensity = -1.;
			//printf("%d : %.3f %.3f %.3f \n", j, filter_data[j], cloud_org->points[j].z, cloud_org->points[j].y);
		}
		else
		{
			cloud_org->points[j].intensity = 0.;
		}
		//printf("%.9f ",cloud_org->points[j].intensity);
		//printf("%.3f ",filter_data[j]);
	}
	printf("\n");
	
	
	VPointCloud::Ptr cloud_lumped(new VPointCloud());
	cloud_lumped->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    cloud_lumped->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    cloud_lumped->height = 1;
    velodyne_pointcloud::PointXYZIR point_prev,point_half;
    {
		//half_index = half_index - min_index;
		//printf("half_index1: %d / %d \n", half_index, outMsg->points.size()); 
		//if (half_index < 0) half_index += outMsg->points.size();
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = cloud_org->points[half_index].ring;
		_point_new.x    = cloud_org->points[half_index].x;
		_point_new.y    = cloud_org->points[half_index].y;
		_point_new.z    = cloud_org->points[half_index].z;
		_point_new.intensity = cloud_org->points[half_index].intensity;
        cloud_lumped->push_back(_point_new);
        //printf("half_index: %d / %d \n", half_index, outMsg->points.size()); 
        point_half.ring = _point_new.ring;
        point_half.x = _point_new.x;
        point_half.y = _point_new.y;
        point_half.z = _point_new.z;
        point_half.intensity = _point_new.intensity;
    }
    
    point_prev.ring = point_half.ring;
    point_prev.x = point_half.x;
    point_prev.y = point_half.y;
    point_prev.z = point_half.z;
    point_prev.intensity = point_half.intensity;
    
#define SEGMENT_DIST_FILTER 0.5
    printf("\n\n");
    for (int j = half_index+1; j < cloud_org->points.size(); j++)
    {
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = cloud_org->points[j].ring;
		_point_new.x    = cloud_org->points[j].x;
		_point_new.y    = cloud_org->points[j].y;
		_point_new.z    = cloud_org->points[j].z;
		_point_new.intensity = cloud_org->points[j].intensity;
		if (fabs(_point_new.x - point_prev.x) < SEGMENT_DIST_FILTER
		    && fabs(_point_new.y - point_prev.y) < SEGMENT_DIST_FILTER
		    && fabs(_point_new.z - point_prev.z) < SEGMENT_DIST_FILTER
		   ){
            cloud_lumped->push_back(_point_new);
            point_prev.ring = _point_new.ring;
            point_prev.x    = _point_new.x;
            point_prev.y    = _point_new.y;
            point_prev.z    = _point_new.z;
            point_prev.intensity = _point_new.intensity;
            if (filter_data[j] < -CURB_ZCROSS_THRESHOLD || filter_data[j] > CURB_ZCROSS_THRESHOLD)
              printf("%d : %.3f %.3f %.3f %.3f %.3f \n", j ,filter_data[j], cloud_org->points[j].z, cloud_org->points[j].x, cloud_org->points[j].y, angle_data[j]);
	    }
	    else
	    {
			//break;
		}
	}
	
	point_prev.ring = point_half.ring;
    point_prev.x = point_half.x;
    point_prev.y = point_half.y;
    point_prev.z = point_half.z;
    point_prev.intensity = point_half.intensity;
	
	printf("\n\n");
	for (int j = half_index-1; j >= 0; j--)
    {
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = cloud_org->points[j].ring;
		_point_new.x    = cloud_org->points[j].x;
		_point_new.y    = cloud_org->points[j].y;
		_point_new.z    = cloud_org->points[j].z;
		_point_new.intensity = cloud_org->points[j].intensity;
        if (fabs(_point_new.x - point_prev.x) < SEGMENT_DIST_FILTER
		    && fabs(_point_new.y - point_prev.y) < SEGMENT_DIST_FILTER
		    && fabs(_point_new.z - point_prev.z) < SEGMENT_DIST_FILTER
		   ){
            cloud_lumped->push_back(_point_new);
            point_prev.ring = _point_new.ring;
            point_prev.x    = _point_new.x;
            point_prev.y    = _point_new.y;
            point_prev.z    = _point_new.z;
            point_prev.intensity = _point_new.intensity;
            if (filter_data[j] < -CURB_ZCROSS_THRESHOLD || filter_data[j] > CURB_ZCROSS_THRESHOLD)
              printf("%d : %.3f %.3f %.3f %.3f %.3f \n", j ,filter_data[j], cloud_org->points[j].z, cloud_org->points[j].x, cloud_org->points[j].y, angle_data[j]);
	    }
	    else
	    {
			//break;
		}
	}
	

	// Finding curb point
//#define CURB_ZHEIGHT_THRESHOLD 0.05
//#define CURB_YDIFF_THRESHOLD 1.5//0.5
	
	VPointCloud::Ptr outMsg_curb(new VPointCloud());
    outMsg_curb->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg_curb->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg_curb->height = 1;
	
	bool _found_negative = false;
	bool _found_positive = false;
	uint8_t _follow_positive = 0;
	uint8_t _follow_negative = 0;
	uint8_t _last_follow_positive = 0;
	uint8_t _last_follow_negative = 0;
	float _min_negative_crossprod, _last_min_negative_crossprod;
	float _max_positive_crossprod, _last_max_positive_crossprod;
	int _min_negative_index, _last_min_negative_index;
	int _max_positive_index, _last_max_positive_index;
	int _min_x_index, _last_min_x_index;
	int _max_x_index, _last_max_x_index;
	float _min_x, _last_min_x;
	float _max_x, _last_max_x;
	int _curb_point_index;
	bool _search_next = false;
	bool _curb_found  = false;
	for (int j = half_index; j < cloud_org->points.size(); j++)
    {
		
		if (filter_data[j] < 0)
		{
			if(!_found_negative){
			  _last_follow_positive = _follow_positive;
			  _found_negative = true;
			  _found_positive = false;
			  _last_min_negative_index     = _min_negative_index;
			  _last_min_negative_crossprod = _min_negative_crossprod;
			  _min_negative_crossprod =  filter_data[j];
			  _min_negative_index = j;
			  
			  //_last_min_x = _min_x;
			  _last_max_x = _max_x;
			  //_last_min_x_index = _min_x_index;
			  _last_max_x_index = _max_x_index;
			  //_min_x = cloud_org->points[j].x;
			  _max_x = cloud_org->points[j].x;
			  //_min_x_index = j;
			  _max_x_index = j;
			  printf("right search found negative %d %d\n", j, _last_follow_positive);
		    }
			_follow_negative++;
			_follow_positive = 0;
			if (filter_data[j] < _min_negative_crossprod) {
			    _min_negative_crossprod = filter_data[j];
			    _min_negative_index = j;
			    //printf("min negative index found: %d...\n",j);
			}
			
			//if (cloud_org->points[j].x < _min_x) {
				//_min_x = cloud_org->points[j].x;
				//_min_x_index = j;
			//}
			if (cloud_org->points[j].x > _max_x) {
				_max_x = cloud_org->points[j].x;
				_max_x_index = j;
			}
			
			if (_last_follow_negative > 6 && _last_follow_positive > 6 && !_search_next)
			{
				bool _match_curb = false;
				if (//cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y < CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y > -CURB_YDIFF_THRESHOLD
				    cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y < CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y > -CURB_YDIFF_THRESHOLD
				   )
				{
					velodyne_pointcloud::PointXYZIR _point_new;
					_point_new.ring = cloud_org->points[_curb_point_index].ring;
					_point_new.x    = cloud_org->points[_curb_point_index].x;
					_point_new.y    = cloud_org->points[_curb_point_index].y;
					_point_new.z    = cloud_org->points[_curb_point_index].z;
					_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
					outMsg_curb->push_back(_point_new);
					printf("MATCH CURB!\n");
					_match_curb = true;
			    }
			    else
			    {
					printf("NO MATCH CURB!\n");
					_search_next = true;
					//continue;
				}
					//int _curb_point_index = _last_min_negative_index+(_max_positive_index - _last_min_negative_index)/2;
					printf("found right!!... at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
					
					
			    //else
			    //{
					printf("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
				//}
				//printf("GIVE UP CURB RIGHT!\n");
				if (_match_curb){
				  _curb_found = true;
				  break;
			    }
			}
		}
		else
		{
			if(!_found_positive){
			  _last_follow_negative = _follow_negative;
			  _found_negative = false;
			  _found_positive = true;
			  _last_max_positive_index     = _max_positive_index;
			  _last_max_positive_crossprod = _max_positive_crossprod;
			  _max_positive_crossprod = filter_data[j];
			  _max_positive_index = j;
			  if (_last_follow_negative > 6){
				_curb_point_index = j;
			  }
			  _search_next = false;
			  
			  _last_min_x = _min_x;
			  //_last_max_x = _max_x;
			  _last_min_x_index = _min_x_index;
			  //_last_max_x_index = _max_x_index;
			  _min_x = cloud_org->points[j].x;
			  //_max_x = cloud_org->points[j].x;
			  _min_x_index = j;
			  //_max_x_index = j;
			  printf("right search found positive %d %d\n", j, _last_follow_negative);
		    }
			_follow_positive++;
			_follow_negative = 0;
			if (filter_data[j] > _max_positive_crossprod) {
			    _max_positive_crossprod = filter_data[j];
			    _max_positive_index = j;
			}
			
			if (cloud_org->points[j].x < _min_x) {
				_min_x = cloud_org->points[j].x;
				_min_x_index = j;
			}
			//if (cloud_org->points[j].x > _max_x) {
				//_max_x = cloud_org->points[j].x;
				//_max_x_index = j;
			//}
			    
			    
			//if (_last_follow_negative > 6 && _follow_positive > 6)
			//{
				//printf("found right!!... at %d : %.3f %.3f %.3f %.3f %d : %.3f %.3f %.3f %.3f \n"
				       //, j
				       //, filter_data[j]
				       //, cloud_org->points[j].z
				       //, cloud_org->points[j].x
				       //, cloud_org->points[j].y
				       
				       //, _min_negative_index
				       //, _min_negative_crossprod
				       //, cloud_org->points[_min_negative_index].z
				       //, cloud_org->points[_min_negative_index].x
				       //, cloud_org->points[_min_negative_index].y
				      //);
				//break;
			//}
			
		}
        
	}
	// TO PREVENT MISS AT END
	if (_last_follow_negative > 6 && _follow_positive > 6)
	{
		printf("MAy MISS at END!\n");
		if (cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD)
		{
			printf("MATCH CURB!\n");
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = cloud_org->points[_curb_point_index].ring;
			_point_new.x    = cloud_org->points[_curb_point_index].x;
			_point_new.y    = cloud_org->points[_curb_point_index].y;
			_point_new.z    = cloud_org->points[_curb_point_index].z;
			_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
			outMsg_curb->push_back(_point_new);
			_curb_found = true;
		}
		else
		{
			printf("NO MATCH CURB!\n");
		}
		printf("found right!!... at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
		printf("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
	}
	
	if (!_curb_found) printf("GIVE UP CURB RIGHT!\n");
	if (_curb_found){
		fprintf(fp,"RIGHT\t%d\t%.3f\t%.3f\n"
		        ,g_info_counter
		        ,cloud_org->points[_curb_point_index].x
		        ,cloud_org->points[_curb_point_index].y
		       );
	}
	
	
	_found_negative = false;
	_found_positive = false;
	_follow_positive = 0;
	_follow_negative = 0;
	_last_follow_positive = 0;
	_last_follow_negative = 0;
	//_min_negative_crossprod =  0.;
	//_max_positive_crossprod = -1.;
	_curb_found = false;
	_search_next = false;
	int _jj ;
	for (int j = half_index; j >= 0; j--)
    {
		_jj = j;
		if (filter_data[j] < 0)
		{
			if(!_found_negative){
			  _last_follow_positive = _follow_positive;
			  _found_negative = true;
			  _found_positive = false;
			  _last_min_negative_index     = _min_negative_index;
			  _last_min_negative_crossprod = _min_negative_crossprod;
			  _min_negative_crossprod =  filter_data[j];
			  _min_negative_index = j;
			  
			  //_last_min_x = _min_x;
			  _last_max_x = _max_x;
			  //_last_min_x_index = _min_x_index;
			  _last_max_x_index = _max_x_index;
			  //_min_x = cloud_org->points[j].x;
			  _max_x = cloud_org->points[j].x;
			  //_min_x_index = j;
			  _max_x_index = j;
			  //printf("left search found negative %d %d\n", j, _last_follow_positive);
		    }
			_follow_negative++;
			_follow_positive = 0;
			if (filter_data[j] < _min_negative_crossprod) {
			    _min_negative_crossprod = filter_data[j];
			    _min_negative_index = j;
			}
			
			//if (cloud_org->points[j].x < _min_x) {
				//_min_x = cloud_org->points[j].x;
				//_min_x_index = j;
			//}
			if (cloud_org->points[j].x > _max_x) {
				_max_x = cloud_org->points[j].x;
				_max_x_index = j;
			}
			
			if (_last_follow_negative > 6 && _last_follow_positive > 6 && !_search_next)
			{
				bool _match_curb = false;
				if (//cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y > -CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y < CURB_YDIFF_THRESHOLD
				    cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD
				    //&& cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y < CURB_YDIFF_THRESHOLD
				    //&& cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y > -CURB_YDIFF_THRESHOLD
				    
				   )
				{
					velodyne_pointcloud::PointXYZIR _point_new;
					_point_new.ring = cloud_org->points[_curb_point_index].ring;
					_point_new.x    = cloud_org->points[_curb_point_index].x;
					_point_new.y    = cloud_org->points[_curb_point_index].y;
					_point_new.z    = cloud_org->points[_curb_point_index].z;
					_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
					outMsg_curb->push_back(_point_new);
					printf("MATCH CURB!\n");
					_match_curb = true;
			    }
			    else
			    {
					printf("NO MATCH CURB!\n");
					_search_next = true;
					//continue;
				}
					//int _curb_point_index = _last_min_negative_index+(_max_positive_index - _last_min_negative_index)/2;
					printf("found left!!...  at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
						   , _max_positive_index
						   , _max_positive_crossprod
						   , cloud_org->points[_min_x_index].z // _max_positive_index
						   , cloud_org->points[_max_positive_index].x
						   , cloud_org->points[_min_x_index].y // _max_positive_index
						   
						   , _last_min_negative_index
						   , _last_min_negative_crossprod
						   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
						   , cloud_org->points[_last_min_negative_index].x
						   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
						   
						   , _curb_point_index
						   , filter_data[_curb_point_index]
						   , cloud_org->points[_curb_point_index].z
						   , cloud_org->points[_curb_point_index].x
						   , cloud_org->points[_curb_point_index].y
						   
						   , _min_x_index
						   , _last_max_x_index
						  );
					
					
			    //else
			    //{
					printf("criterion:  %.3f %.3f\n"
					       //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					       //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					       , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					       , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					      );
				//}
				//printf("GIVE UP CURB LEFT!\n");
				if (_match_curb){
				  _curb_found = true;
				  break;
			    }
			}
		}
		else
		{
			if(!_found_positive){
			  _last_follow_negative = _follow_negative;
			  _found_negative = false;
			  _found_positive = true;
			  _last_max_positive_index     = _max_positive_index;
			  _last_max_positive_crossprod = _max_positive_crossprod;
			  _max_positive_crossprod = filter_data[j];
			  _max_positive_index = j;
			  if (_last_follow_negative > 6){
				_curb_point_index = j;
			  }
			  _search_next = false;
			  
			  _last_min_x = _min_x;
			  //_last_max_x = _max_x;
			  _last_min_x_index = _min_x_index;
			  //_last_max_x_index = _max_x_index;
			  _min_x = cloud_org->points[j].x;
			  //_max_x = cloud_org->points[j].x;
			  _min_x_index = j;
			  //_max_x_index = j;
			  //printf("left search found positive %d %d\n", j, _last_follow_negative);
		    }
			_follow_positive++;
			_follow_negative = 0;
			if (filter_data[j] > _max_positive_crossprod) {
			    _max_positive_crossprod = filter_data[j];
			    _max_positive_index = j;
			}
			
			if (cloud_org->points[j].x < _min_x) {
				_min_x = cloud_org->points[j].x;
				_min_x_index = j;
			}
			//if (cloud_org->points[j].x > _max_x) {
				//_max_x = cloud_org->points[j].x;
				//_max_x_index = j;
			//}
			
			//if (_last_follow_negative > 6 && _follow_positive > 6)
			//{
				//printf("found left!! ... at %d : %.3f %.3f %.3f %.3f %d : %.3f %.3f %.3f %.3f \n"
				       //, j
				       //, filter_data[j]
				       //, cloud_org->points[j].z
				       //, cloud_org->points[j].x
				       //, cloud_org->points[j].y
				       
				       //, _min_negative_index
				       //, _min_negative_crossprod
				       //, cloud_org->points[_min_negative_index].z
				       //, cloud_org->points[_min_negative_index].x
				       //, cloud_org->points[_min_negative_index].y
				      //);
				//break;
			//}
			
		}

	}
	if (_last_follow_negative > 6 && _follow_positive > 6 )//&& j == cloud_org->points.size() - 1)
    {
		printf("MAy MISS at END! j=%d size=%d\n", _jj , cloud_org->points.size());
		if (cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z > CURB_ZHEIGHT_THRESHOLD)
		{
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = cloud_org->points[_curb_point_index].ring;
			_point_new.x    = cloud_org->points[_curb_point_index].x;
			_point_new.y    = cloud_org->points[_curb_point_index].y;
			_point_new.z    = cloud_org->points[_curb_point_index].z;
			_point_new.intensity = 0;//cloud_org->points[_curb_point_index].intensity;
			outMsg_curb->push_back(_point_new);
			printf("MATCH CURB!\n");
			_curb_found = true;
		}
		else
		{
			printf("NO MATCH CURB!\n");
		}
		printf("found left!!...  at \n%d : %.3f %.3f %.3f %.3f \n%d : %.3f %.3f %.3f %.3f == \n%d : %.3f %.3f %.3f %.3f \n %d %d \n"
					   , _max_positive_index
					   , _max_positive_crossprod
					   , cloud_org->points[_min_x_index].z // _max_positive_index
					   , cloud_org->points[_max_positive_index].x
					   , cloud_org->points[_min_x_index].y // _max_positive_index
					   
					   , _last_min_negative_index
					   , _last_min_negative_crossprod
					   , cloud_org->points[_last_max_x_index].z // _last_min_negative_index
					   , cloud_org->points[_last_min_negative_index].x
					   , cloud_org->points[_last_max_x_index].y // _last_min_negative_index
					   
					   , _curb_point_index
					   , filter_data[_curb_point_index]
					   , cloud_org->points[_curb_point_index].z
					   , cloud_org->points[_curb_point_index].x
					   , cloud_org->points[_curb_point_index].y
					   
					   , _min_x_index
					   , _last_max_x_index
					  );
					  
		printf("criterion:  %.3f %.3f\n"
					   //, cloud_org->points[_max_positive_index].z - cloud_org->points[_last_min_negative_index].z
					   //, cloud_org->points[_max_positive_index].y - cloud_org->points[_last_min_negative_index].y
					   , cloud_org->points[_min_x_index].z - cloud_org->points[_last_max_x_index].z
					   , cloud_org->points[_min_x_index].y - cloud_org->points[_last_max_x_index].y
					  );
		
	}
	if (!_curb_found) printf("GIVE UP CURB LEFT!\n");
	
	if (_curb_found){
		fprintf(fp,"LEFT\t%d\t%.3f\t%.3f\n"
		        ,g_info_counter
		        ,cloud_org->points[_curb_point_index].x
		        ,cloud_org->points[_curb_point_index].y
		       );
	}
	g_info_counter++;
	
	pub_curb.publish(outMsg_curb);
	
	// Method II for (transformaton matrix) Affine transformation
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  
    // Define a translation of 2.5 meters on the z axis.
    transform_1.translation() << -3.0, 0.0, 0;
  
    // The same rotation matrix as before; tetha radians arround Y axis
    //transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
  
    // The same rotation matrix as before; tetha radians arround X axis
    //transform_2.rotate (Eigen::AngleAxisf (thetaX, Eigen::Vector3f::UnitX()));
  
    // perform transformation using 
    //pcl::transformPointCloud(*outMsg,*cloud_transformed,transform_1);
    
    // append transformed packet data to end of output message
	//cloud_lumped->points.insert(cloud_lumped->points.end(),
	//					 cloud_transformed->points.begin(),
	//					 cloud_transformed->points.end());
	//cloud_lumped->width += cloud_transformed->points.size();
    
    //pub.publish(outMsgs[0]);
    //pub.publish(cloud_lumped);
    pub.publish(cloud_org);
    //pub.publish(outMsg);
    //pub.publish(cloud_transformed_pcl);
    pub_transformed.publish(outMsg_high);
    return;
    

}

int main(int argc, char** argv) 
{
#ifdef WITH_REDIS
	redisContext *c;
    redisReply *reply;
    const char *hostname = "127.0.0.1";//(argc > 1) ? argv[1] : "127.0.0.1";
    int port = 6379;//(argc > 2) ? atoi(argv[2]) : 6379;

    struct timeval timeout = { 1, 500000 }; // 1.5 seconds
    c = redisConnectWithTimeout(hostname, port, timeout);
    if (c == NULL || c->err) {
        if (c) {
            printf("Redis Connection error: %s\n", c->errstr);
            redisFree(c);
        } else {
            printf("Redis Connection error: can't allocate redis context\n");
        }
        exit(1);
    }
#endif
	
	ros::init(argc, argv, "curb_detect_node");
    ros::NodeHandle nh("~");//nh;//("~");
    ros::Rate r(10);
    
    nh.getParam("debug_flag_0",g_debug_test_0);
    nh.getParam("debug_flag_1",g_debug_test_1);
    nh.getParam("debug_flag_2",g_debug_test_2);
    nh.getParam("debug_flag_3",g_debug_test_3);
    nh.getParam("debug_flag_left",g_debug_test_4);
    nh.getParam("debug_flag_right",g_debug_test_5);
    nh.getParam("process_layer",g_flag_process_ring);
    nh.getParam("simul_until",g_flag_simul_until);
    
    //ROS_INFO_STREAM("debug_flag_0" << g_debug_test_0);
    
    {
		//char ts[128];
		//sprintf
        ROS_INFO("debug_flag_0 = %d\n",g_debug_test_0);
        ROS_INFO("debug_flag_1 = %d\n",g_debug_test_1);
        ROS_INFO("debug_flag_2 = %d\n",g_debug_test_2);
        ROS_INFO("debug_flag_3 = %d\n",g_debug_test_3);
        ROS_INFO("debug_flag_left  = %d\n",g_debug_test_4);
        ROS_INFO("debug_flag_right = %d\n",g_debug_test_5);
        ROS_INFO("process_layer = %d\n",g_flag_process_ring);
        ROS_INFO("simul_until = %f\n",g_flag_simul_until);
    }
    //ros::Subscriber sub = nh.subscribe ("velodyne_packets", 1 , cloud_cb);
    //ros::Subscriber sub = nh.subscribe ("velodyne_points", 10 , cloud_cb_2);
    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 10 , cloud_cb_3);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>( "/filtered_points", 10);
    pub_debug = nh.advertise<sensor_msgs::PointCloud2>( "/debug_points", 10);
    pub_curb = nh.advertise<sensor_msgs::PointCloud2>( "/curb_points", 10);
    pub_transformed = nh.advertise<sensor_msgs::PointCloud2>( "/transformed_points", 10);
    
    fp=fopen("Record.txt","w");
    if (g_flag_simul_until == 0)
    {
    while (ros::ok()){ros::spinOnce();}//ROS_INFO("The time is: %f", ros::Time::now().toSec());r.sleep();}//ROS_INFO_STREAM("debug_flag_0" << g_debug_test_0);r.sleep();}
    }
    else
    {
		while (ros::ok() && ros::Time::now().toSec() < g_flag_simul_until){ros::spinOnce();}
	}
    fclose(fp);
    
    return 0;
}
