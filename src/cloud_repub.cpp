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

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub;

void cloud_cb (const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
	//ROS_INFO_STREAM("Yo, whats up I got data!");
	VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = "filtered_velodyne";
    pub.publish(outMsg);
    
}

void cloud_cb_2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    VPointCloud::Ptr cloud(new VPointCloud());
    VPointCloud::Ptr cloud_org(new VPointCloud());
    VPointCloud::Ptr outMsg(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud);
    
    
    outMsg->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg->height = 1;
    
    printf("outMsg size: %d\n", outMsg->points.size());
    
    printf("Rings: %d %d \n", cloud_msg->row_step, cloud_msg->width);
    
    bool _first = true;
    float first_angle, last_angle, prev_angle;
    float min_angle = -90;
    size_t min_index, cur_index; cur_index = 0;
    
    bool _invalid_noisy_data = false;
    
    for (size_t next = 0; next < cloud->points.size(); ++next)
    {
	    //printf(" %d,", (*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()).ring  );	
	    //velodyne_pointcloud::PointXYZIR *_point = cloud->points.begin();// = *((*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()));
	    velodyne_pointcloud::PointXYZIR _point = cloud->at(next);
		//printf(" %d,",_point.ring);
		//if ( 3 <= _point.ring &&  _point.ring <= 15){
		
		if ( _point.ring == 7){ // 7 10
		    
		    
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			//_point_new.intensity = (float)25.0;//_point.intensity;
			_point_new.intensity = _point.intensity;
			float _angle  = atan2f(_point.y,_point.x)*180/3.1459;
			//if (-90 < _angle && _angle < 90){ 
			if (-55 < _angle && _angle < 55){ 
			    
			    //printf(" %f,%f,",_point.x,_point.y);
			    //printf(" %d,%f,",next,_angle);
			    //printf(" %f,",_point_new.intensity);
			    //if (-45 < _angle && _angle < 45) _point_new.intensity = 100;
			    //if (-10 < _angle && _angle < 10) _point_new.intensity = 200;
			    outMsg->push_back(_point_new);
			    //printf(" %f,",sqrt(_point.x*_point.x + _point.y*_point.y + _point.z*_point.z));
			    if (min_angle < _angle) 
			    {
			      min_angle = _angle;
			      min_index = cur_index;
			    }
			    cur_index++;
		    }
			//printf(" %f,",sqrt(_point.x*_point.x + _point.y*_point.y));
			//printf(" %f,",atan2f(_point.y,_point.x)*180/3.1459);
			//printf(" %f,",_point.z);
			//printf(" %f,",_point.x);
			
			if (_first){
		      _first = false;
		      first_angle = _angle;
		      
		    }
		    else
		    {
				if (false)// TO BE REMOVED!!! //(fabs(_angle - last_angle) > 10)
				{
					_invalid_noisy_data = true;
					return;
				}
			}
			last_angle = _angle;
		}
	}
	
	printf("outMsg size: %d\n", outMsg->points.size());
	printf("min_angle : %f min_index: %d\n", min_angle, min_index);
	
	cloud_org->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    cloud_org->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    cloud_org->height = 1;
    _first = true;
	
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
		
		if (_first){
		      _first = false;
		      
		      
		}
		else
		{
		    if (false)//((_angle - prev_angle) < 0)
			{
				_invalid_noisy_data = true;
				return;
		    }
		}
		prev_angle = _angle;
	}
	
	
	float radial_data[cloud_org->points.size()];
	float filter_data[cloud_org->points.size()];
	
	for (size_t next = 0; next < cloud_org->points.size(); next++)
    {
		  //if (next < 450) cloud_org->points[next].intensity = 25;
		  //else cloud_org->points[next].intensity = 100;
		//cloud_org->points[next].intensity = next;
		  //float _angle  = atan2f(cloud_org->points[next].y,cloud_org->points[next].x)*180/3.1459;
		  //printf(" %d,%f,",next,_angle);
		
		float _radial = sqrtf(  cloud_org->points[next].x*cloud_org->points[next].x 
		                      + cloud_org->points[next].y*cloud_org->points[next].y
		                      + cloud_org->points[next].z*cloud_org->points[next].z
		                     );
		//cloud_org->points[next].intensity = _radial;
		radial_data[next] = _radial;
		//radial_data[next] = cloud_org->points[next].x;
	}
	
	
	int t_bef, t_after;
	int tau = 15; //  15  5
	
	//printf("\n");
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
		float _zvec1 =  cloud_org->points[j].z - cloud_org->points[t_bef].z;
		
		float _xvec2 = -cloud_org->points[j].x + cloud_org->points[t_after].x;
		float _yvec2 = -cloud_org->points[j].y + cloud_org->points[t_after].y;
		float _zvec2 = -cloud_org->points[j].z + cloud_org->points[t_after].z;
		
		float _cross_i = _yvec1*_zvec2 - _zvec1*_yvec2;
		float _cross_j = _zvec1*_xvec2 - _xvec1*_zvec2;
		float _cross_k = _xvec1*_yvec2 - _yvec1*_xvec2;
		
		filter_data[j]     = sqrtf(_cross_i*_cross_i + _cross_j*_cross_j + _cross_k*_cross_k);
		//filter_data[j]     =  _cross_k;
		
		//cloud_org->points[j].intensity = filter_data[j];
		//cloud_org->points[j].intensity = _cross_k;
		
		//if (filter_data[j] > 0.05)
		//{
		//	cloud_org->points[j].intensity = 1.;
		//}
		//else
		//{
		//	cloud_org->points[j].intensity = 0.;
		//}
		//printf("%.9f ",cloud_org->points[j].intensity);
		//printf("%.3f ",filter_data[j]);
	}
	//printf("\n");
	
	float filter_data2[cloud_org->points.size()];
	
	printf("\n");
	for (int j = 0; j < cloud_org->points.size(); j++)
    {
		t_bef   = j - tau; if (t_bef < 0) t_bef = 0;
		t_after = j + tau; if (t_after > cloud_org->points.size()-1) t_after = cloud_org->points.size()-1;
		
		filter_data2[j]     =  filter_data[t_bef] + filter_data[t_after];
		
		cloud_org->points[j].intensity = filter_data2[j];
		printf("%.3f ",cloud_org->points[j].intensity);
	}
	printf("\n");
	
	printf("\n%f,%f\n", first_angle, last_angle); 
	float _first_cloud_org_angle_degree = atan2f(cloud_org->points[0].y,cloud_org->points[0].x)*180/3.1459;
	float _last_cloud_org_angle_degree  = atan2f(cloud_org->points[cloud_org->points.size()-1].y, cloud_org->points[cloud_org->points.size()-1].x)*180/3.1459;
	printf("\n%f,%f\n", _first_cloud_org_angle_degree
	, _last_cloud_org_angle_degree
	);
	
	// filter bug on the velodyne firmware may be...
	if (_first_cloud_org_angle_degree - _last_cloud_org_angle_degree < 20.) {
		printf("NOT PUBLISHING!!! \n"); 
		return;
	}
	
	//printf("\n%f,%f\n", sqrt(cloud_org->points[0].x*cloud_org->points[0].x + cloud_org->points[0].y*cloud_org->points[0].y)
	//, sqrt(cloud_org->points[cloud_org->points.size()-1].x*cloud_org->points[cloud_org->points.size()-1].x + cloud_org->points[cloud_org->points.size()-1].y*cloud_org->points[cloud_org->points.size()-1].y)
	//
	//);
    //printf("\n");
    
    /*_first = false;
    
    for (size_t next = 0; next < cloud->points.size(); ++next)
    {
	    //printf(" %d,", (*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()).ring  );	
	    //velodyne_pointcloud::PointXYZIR *_point = cloud->points.begin();// = *((*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()));
	    velodyne_pointcloud::PointXYZIR _point = cloud->at(next);
		//printf(" %d,",_point.ring);
		//if ( 3 <= _point.ring &&  _point.ring <= 15){
		
		if ( _point.ring == 15){ // 7
		    
		    
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			_point_new.intensity = _point.intensity;
			float _angle  = atan2f(_point.y,_point.x)*180/3.1459;
			//if (-45 < _angle && _angle < 45){ 
			    outMsg->push_back(_point_new);
			    //printf(" %f,%f,",_point.x,_point.y);
			    //printf(" %d,%f,",next,_angle);
		    //}
			//printf(" %f,",sqrt(_point.x*_point.x + _point.y*_point.y));
			//printf(" %f,",atan2f(_point.y,_point.x)*180/3.1459);
			//printf(" %f,",_point.z);
			//printf(" %f,",_point.x);
			
			if (!_first){
		      _first = true;
		      first_angle = _angle;
		    }
			last_angle = _angle;
		}
	}
	printf("\n%f,%f\n", first_angle, last_angle);*/
    
    //pub.publish(outMsg);
    pub.publish(cloud_org);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "cloud_repub_node");
    ros::NodeHandle nh;//("~");
    ros::Rate r(10);
    
    //ros::Subscriber sub = nh.subscribe ("velodyne_packets", 1 , cloud_cb);
    ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , cloud_cb_2);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>( "filtered_points", 1);
    
    while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
    return 0;
}
