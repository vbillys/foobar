#ifndef COMMON_H_CURB_DETECT
#define COMMON_H_CURB_DETECT

#include "data_types.h"
#include <ros/ros.h>
#include <stdlib.h> 

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>



float normal_pdf(float x, float m, float s)
;

int compareIndexedAngle (const void * a, const void * b)
;

#ifndef PI
#define PI 3.14159265
#endif

#endif
