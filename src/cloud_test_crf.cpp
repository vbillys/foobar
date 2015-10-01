//#define NDEBUG
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

//#include <pcl/kdtree/kdtree_flann.h>
#include <armadillo>
#include <stdlib.h> 
#include <initializer_list>

#include "cloud_util.h"
#include "debug_print.hpp"
#include "ring_process.h"
#include "common.h"
#include "data_types.h"

#include "data/velo_data.h"

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

ros::Publisher  pub, pub_transformed;
float g_angle_data_ring[16][1000];
float g_angle_data_ring_org[16][1000];

typedef struct {
	float y0;
	float y1;
	float x0;
	float x1;
} CellBoundary2D;

typedef struct {
	arma::fmat points;
	CellBoundary2D boundary;
} RingCell;

//typedef std::vector<RingCell> RingCells;
//typedef std::vector<RingCells> AllRingCells;
typedef arma::field<CellBoundary2D> CellBoundaries;
//typedef arma::field<CellBoundaries> AllCellBoundaries(CellBoundaries);
typedef std::vector<arma::fmat> CellPoints;
typedef std::vector<CellPoints> AllCellPoints;

typedef arma::Row<float> CellBoundary2DRow;
typedef arma::Mat<float> CellPoints3DMat;

void cloud_cb_2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	arma::fmat A;

    A << 1 << 3 << 5 << arma::endr
      << 2 << 4 << 6 << arma::endr;
	
	//std::cout << A << std::endl;
	
	CellPoints _1st_row; _1st_row.push_back(A);
	CellPoints _2nd_row; _2nd_row.push_back(A);
	std::vector<CellPoints> _field_rst(16);// = _1st_row;//arma::join_rows(_1st_row , _2nd_row);
	_field_rst[0] = _1st_row;
	_field_rst[1] = _2nd_row;
	//fmat _tmat = _field_rst(0,0).points;
	//_field_rst(0,0).points(0,0) = 1;// << arma::endr;
	//_field_rst(0,1).points(0,0) = 2;// << arma::endr;
	
	//CellPoints _tpoints;
	//_tpoints = _field_rst(0);
	//_tpoints.print();
	//_tpoints = _field_rst(1);
	//_tpoints.print();
	
	//_field_rst(1,0).points.print();
	//_field_rst = arma::join_rows(_field_rst,_field_rst(0));
	//_field_rst.insert_rows(_field_rst(0),2);
	_1st_row[0].print();
	
	arma::field<arma::fmat> _memek(16);
	_memek(15)= A; _memek(1) = A;
	
	
	CellBoundaries _test_bound;
	arma::field<CellBoundaries> _test_boundaries(CellBoundaries);
	arma::field<CellBoundaries> _test_fixed_boundaries(16);
	
	CellBoundary2DRow _bangsat(4);_bangsat.fill(1);
	CellBoundary2DRow _kentut(4);_kentut.fill(2);
	//arma::field<CellBoundary2D> _semak(arma::field<CellBoundary2D>);
	//arma::field<arma::fmat> _semak;
	arma::fmat _semak;
	//_semak.print();
	//_semak = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	//_semak << _bangsat;
	_semak = join_cols(_kentut,_bangsat);
	CellBoundary2DRow _krui = _semak.row(1);
	_semak.print();_krui.print();
	_memek(1) = _semak;
	_memek.print();
	_memek(1).print();
	
	arma::field<arma::fmat> _clentit = _memek;
	_clentit.print();
	
	//_test_fixed_boundaries(1) = _test_bound;
	//std::cout << _test_bound.x0 << std::endl;
	//_test_fixed_boundaries.print();
	
	VPointCloud::Ptr cloud(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud);
    
    VPointCloud::Ptr cloud_org(new VPointCloud());
    VPointCloud::Ptr outMsg(new VPointCloud());
    VPointCloud::Ptr cloud_transformed(new VPointCloud());
    
   
    
    
    outMsg->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    outMsg->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    outMsg->height = 1;
    
    printf("outMsg size: %lu\n", outMsg->points.size());
    
    printf("Rings: %d %d \n", cloud_msg->row_step, cloud_msg->width);
    
    
    bool _first = true;
    float first_angle, last_angle, prev_angle;
    float min_angle = -90;
    size_t min_index, cur_index; cur_index = 0;
    
    bool _invalid_noisy_data = false;
    int ring_no = 7;
    
    for (size_t next = 0; next < cloud_transformed->points.size(); ++next)
    {
	    //printf(" %d,", (*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()).ring  );	
	    //velodyne_pointcloud::PointXYZIR *_point = cloud->points.begin();// = *((*velodyne_pointcloud::PointXYZIR)(cloud->points.begin()));
	    velodyne_pointcloud::PointXYZIR _point = cloud_transformed->at(next);
		//printf(" %d,",_point.ring);
		//if ( 3 <= _point.ring &&  _point.ring <= 15){
		
		if ( _point.ring == ring_no){ // 7 10
		    
		    
			velodyne_pointcloud::PointXYZIR _point_new;
			_point_new.ring = _point.ring;
			_point_new.x    = _point.x;
			_point_new.y    = _point.y;
			_point_new.z    = _point.z;
			//_point_new.intensity = (float)25.0;//_point.intensity;
			_point_new.intensity = _point.intensity;
			float _angle  = atan2f(_point.y,_point.x)*180/3.1459;
			if (-90 < _angle && _angle < 90){ 
			//if (-55 < _angle && _angle < 55){ 
			    
			    //printf(" %f,%f,",_point.x,_point.y);
			    //printf(" %d,%f,",next,_angle);
			    //printf(" %f,",_point_new.intensity);
			    //if (-45 < _angle && _angle < 45) _point_new.intensity = 100;
			    //if (-10 < _angle && _angle < 10) _point_new.intensity = 200;
			    outMsg->push_back(_point_new);
			    g_angle_data_ring[ring_no][cur_index] = _angle;
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
	
	printf("outMsg size: %lu\n", outMsg->points.size());
	printf("min_angle : %f min_index: %lu\n", min_angle, min_index);
	
	
	
	cloud_org->header.stamp = pcl_conversions::toPCL(cloud_msg->header).stamp;
    cloud_org->header.frame_id = cloud->header.frame_id;//"filtered_velodyne";
    cloud_org->height = 1;
    _first = true;
    
    bool _sort_required = false;
    IndexedAngle angle_sorted[outMsg->points.size()];
	
	for (size_t next = 0; next < outMsg->points.size(); next++)
    {
		size_t _index_reindex = next + min_index;
		if (_index_reindex >= outMsg->points.size()) _index_reindex -= outMsg->points.size();
		//if (next < 450) outMsg->points[_index_reindex].intensity = 25;
		//else outMsg->points[_index_reindex].intensity = 100;
		velodyne_pointcloud::PointXYZIR _point_new;
		_point_new.ring = outMsg->points[_index_reindex].ring;
		_point_new.x    = 0;//outMsg->points[_index_reindex].x;
		_point_new.y    = outMsg->points[_index_reindex].y;
		_point_new.z    = outMsg->points[_index_reindex].z;
		_point_new.intensity = outMsg->points[_index_reindex].intensity;
		cloud_org->push_back(_point_new);
		
		//float _angle  = atan2f(_point_new.y,_point_new.x)*180/3.1459;
		
		float _angle  = g_angle_data_ring[ring_no][_index_reindex];
		g_angle_data_ring_org [ring_no][next] = _angle;
		angle_sorted[next].angle = _angle;
		angle_sorted[next].index = _index_reindex;
		
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
		
		if (prev_angle < _angle)
		{
		    _sort_required = true;
	    }
		
		prev_angle = _angle;
	}
	
	if (_sort_required)
	{
		cloud_org->clear();
		cloud_org->height = 1;
		qsort (angle_sorted
		       ,outMsg->points.size()
		       ,sizeof(IndexedAngle)
		       ,compareIndexedAngle
		      );
		debug_print_6("\n");
		debug_print_6("Resorting: \n");
		//min_height_index = angle_sorted[min_height_index].index;
		for (int next = 0; next < outMsg->points.size(); next++)
        {
			int _index_reindex = angle_sorted[next].index;
			float _angle  = g_angle_data_ring[ring_no][_index_reindex];
			//float _angle  =  angle_sorted[_index_reindex].angle;
		    g_angle_data_ring_org [ring_no][_index_reindex] = _angle;
		    debug_print_6("%d : %.3f \n", next, _angle);
		    velodyne_pointcloud::PointXYZIR _point_new;
		    _point_new.ring = outMsg->points[_index_reindex].ring;
		    _point_new.x    = 0;//outMsg->points[_index_reindex].x;
		    _point_new.y    = outMsg->points[_index_reindex].y;
		    _point_new.z    = outMsg->points[_index_reindex].z;
		    _point_new.intensity = next;//outMsg->points[_index_reindex].intensity;
		    cloud_org->push_back(_point_new);
		    _point_new.intensity = next;
		    //g_outMsg_debug2->push_back(_point_new);
		    
		    //if (_index_reindex == min_height_index_t)
		    //  min_height_index = next;
		}
		debug_print_6("\n");
	}
	
	pub_transformed.publish(cloud_transformed);
	pub.publish(cloud_org);
	
	
	
	return;
	
	
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

class CloudProcessor
{
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;
  ros::Publisher  pub_transformed;
  
public:
  CloudProcessor()
  {

    sub = nh.subscribe ("velodyne_points", 1 , &CloudProcessor::cloudCb, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>( "filtered_points", 1);
    pub_transformed = nh.advertise<sensor_msgs::PointCloud2>( "/transformed_points", 10);
  }

  ~CloudProcessor()
  {
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
      velodyne_process_data::VLP16PointCloud pcloud(cloud_msg);
      
      
      VPointCloud::Ptr cloud(new VPointCloud());
      pcl::fromROSMsg(*cloud_msg , *cloud);
      
      cloud_util::CloudUtil cu(cloud);
    
      VPointCloud::Ptr cloud_transformed(new VPointCloud());
      EulerTransformParameters _transform = {0,0,1.5,-2*PI/180,22*PI/180,0};
      pcloud.transform(_transform);

      cu.transformCloud(       cloud
                               , cloud_transformed
                               , _transform.x           // x translate
                               , _transform.y           // y translate
                               , _transform.z          // z translate
                               , _transform.theta_x  // roll
                               , _transform.theta_y  // pith
                               , _transform.theta_z           // supposedly, yaw..?
                              );
      
      VPointCloud::Ptr outMsg_debug(new VPointCloud());
      VPointCloud::Ptr outMsg_high (new VPointCloud());
      
      
      int8_t _iring_tobe_processed[] = {4,5,6,7,8};
	  std::vector<int8_t> _ring_tobe_processed (_iring_tobe_processed , _iring_tobe_processed+sizeof(_iring_tobe_processed)/sizeof(_iring_tobe_processed[0]));
	  
	  
      cu.rangeSelectCloud(         cloud_transformed
                                   , outMsg_debug
                                   , outMsg_high
                                   , _ring_tobe_processed//- 1
                                   ,   0.5
                                   , -90
                                   ,  90
                                  );
      pub.publish(outMsg_debug);
      pub_transformed.publish(outMsg_high);
  }
};

bool GlobalDebugData::g_debug_test_0 = DEBUG_TEST_0;
bool GlobalDebugData::g_debug_test_1 = DEBUG_TEST_1;
bool GlobalDebugData::g_debug_test_2 = DEBUG_TEST_2;
bool GlobalDebugData::g_debug_test_3 = DEBUG_TEST_3;
bool GlobalDebugData::g_debug_test_4 = DEBUG_TEST_4;
bool GlobalDebugData::g_debug_test_5 = DEBUG_TEST_5;
bool GlobalDebugData::g_debug_test_6 = DEBUG_TEST_6;

namespace globaldebugdata{
	BasicFilter::Ptr DebugFlags::g_debug_test_0(new BasicFilter(DEBUG_TEST_0));
	BasicFilter::Ptr DebugFlags::g_debug_test_1(new BasicFilter(DEBUG_TEST_1));
	BasicFilter::Ptr DebugFlags::g_debug_test_2(new BasicFilter(DEBUG_TEST_2));
	BasicFilter::Ptr DebugFlags::g_debug_test_3(new BasicFilter(DEBUG_TEST_3));
	BasicFilter::Ptr DebugFlags::g_debug_test_4(new BasicFilter(DEBUG_TEST_4));
	BasicFilter::Ptr DebugFlags::g_debug_test_5(new BasicFilter(DEBUG_TEST_5));
	BasicFilter::Ptr DebugFlags::g_debug_test_6(new BasicFilter(DEBUG_TEST_6));
}
void semak_main();
int main(int argc, char** argv) 
{
    semak_main();
	GlobalDebugData::setDebugTest6Flag(0);
	
	ros::init(argc, argv, "cloud_process_node");
    
    CloudProcessor cp;
    
    while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
    
    return 0;
}


