#ifndef DATA_TYPES_CURB_DETECT
#define DATA_TYPES_CURB_DETECT

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

typedef struct {
	double x, y, z, theta_x, theta_y, theta_z;
} EulerTransformParameters;

typedef struct {
	float angle;
	int   index;
} IndexedAngle;

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;


typedef IndexedAngle VAngleDataOrg[2000];
typedef float VAngleData[32000];

typedef VAngleDataOrg VAngleDataOrgRings[16];
typedef int           NoOfPointsRings[16];
typedef VPointCloud::Ptr   RingPointCloud[16];
typedef int           MinIndex[16];
typedef float         MinAngle[16];


struct  OrganizedPointCloud{
	VAngleDataOrgRings angle_data_orgs;
	NoOfPointsRings    no_of_points;
    RingPointCloud     ring_point_cloud;
};

#endif
