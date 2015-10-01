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


bool g_debug_test_6 = 0;
#define debug_print_6( ...) \
            do { if (g_debug_test_6) fprintf(stdout,  __VA_ARGS__); } while (0)

    //ros::NodeHandle nh;//("~");
    //ros::Rate r(10);
    
    ////ros::Subscriber sub = nh.subscribe ("velodyne_packets", 1 , cloud_cb);
    //ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , cloud_cb_2);
    
    //pub = nh.advertise<sensor_msgs::PointCloud2>( "filtered_points", 1);
    //pub_transformed = nh.advertise<sensor_msgs::PointCloud2>( "/transformed_points", 10);

void cloud_cb (const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
	
	VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = "filtered_velodyne";
    pub.publish(outMsg);
}

    //test
    static int _test = 0;
    _test++;ROS_INFO("test counter : %d",_test);
    if (_test > 10) {ROS_WARN("HAHA SHUTTING DOWN BITHCH!!");ros::shutdown();}

    if (angle_max < angle_min) {
		ROS_DEBUG_STREAM ( "angle range invalid quitting...!" << std::endl << "at:" << __FUNCTION__ << " " << __FILE__ << " " << __LINE__ << " " << "compiled on:" << __DATE__ << __TIME__ << "modified:" << __TIMESTAMP__);
		ROS_INFO_STREAM ( "angle range invalid quitting...!" << std::endl << "at:" << __FUNCTION__ << " " << __FILE__ << " " << __LINE__ << " " << "compiled on:" << __DATE__ << __TIME__ << "modified:" << __TIMESTAMP__);
		ros::shutdown();
	}
	

//ROS_INFO_STREAM("Yo, whats up I got data!");     

                  
