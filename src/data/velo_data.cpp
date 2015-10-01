/**
 * @file velo_data.cpp
 * @brief Represents data from Velodyne LIDAR 16 layers
 *        Data are organized based on laser rays clockwise and its layer has
 *        ring no. Cartesian coordinate frame is defined as:
 *          - x axis is pointing forward
 *          - y axis is pointing left
 *          - z axis is pointing upward
 * @author Vincensius Billy Saputra <saputravb@i2r.a-star.edu.sg>
 * @version 0.01
 * @date 2015-08-29
 */

/* Copyright (C) 
 * 2015 - Vincensius Billy Saputra <saputravb@i2r.a-star.edu.sg>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 */


#include "velo_data.h"
#include <boost/foreach.hpp>

//map gv "+gP
//kentut


void gampanggampangsaja(float & input
		, float & output
		, int additional_options
		)
{
	int a, b, c;
	input = a + b + c;


}

namespace velodyne_process_data{
	/**
	 * @brief
	 *
	 * @param cloud_msg
	 */
	VLP16PointCloud::VLP16PointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	:	ros_data_ (const_cast<sensor_msgs::PointCloud2ConstPtr&>(cloud_msg))
	{
		allocateDataMem();
		pcl::fromROSMsg(*ros_data_ , *p_raw_data_);
		initHeaderPCs();
		//convertToRingsData();

		//when initialize the contruct doesnot have pcl header format yet
		cu_.setCloud(p_raw_data_);
	}


	/**
	 * @brief 
	 */
	VLP16PointCloud::~VLP16PointCloud()
	{}

	void VLP16PointCloud::allocateDataMem()
	{
		p_raw_data_.reset(new VPointCloud());
		p_raw_transformed_data_.reset(new VPointCloud());
		p_roi_data_.reset(new VPointCloud());
		p_not_roi_data_.reset(new VPointCloud());

		BOOST_FOREACH ( VPointCloud::Ptr &  _cloud, org_data_.ring_point_cloud){
		  _cloud.reset(new VPointCloud());
		}
	}

	void VLP16PointCloud::initHeaderPCs()
	{
		initHeader(p_not_roi_data_);
		initHeader(p_roi_data_);

		initHeaderPCsOrg();

	}

	void VLP16PointCloud::initHeaderPCsOrg()
	{
		BOOST_FOREACH ( VPointCloud::Ptr &  _cloud, org_data_.ring_point_cloud){
			initHeader(_cloud);
		}
	}

	/**
	 * @brief 
	 *
	 * @param cloud
	 */
	void VLP16PointCloud::initHeader(VPointCloud::Ptr& cloud)
	{
		cloud->clear();
		cloud->header.stamp = p_raw_data_->header.stamp;
		cloud->header.frame_id = p_raw_data_->header.frame_id;
		cloud->height = 1;
	}

	void VLP16PointCloud::convertToRingsData()
	{
		initHeaderPCsOrg();
		BOOST_FOREACH(VPoint _point, *p_raw_data_){
			org_data_.ring_point_cloud[_point.ring]->push_back(_point);
		}
	}

	void VLP16PointCloud::convertToRingsDataFromTransformed()
	{
		initHeaderPCsOrg();
		BOOST_FOREACH(VPoint _point, *p_raw_transformed_data_){
			org_data_.ring_point_cloud[_point.ring]->push_back(_point);
		}
	}

	void VLP16PointCloud::convertToRingsDataFromROI()
	{
		initHeaderPCsOrg();
		BOOST_FOREACH(VPoint _point, *p_roi_data_){
			org_data_.ring_point_cloud[_point.ring]->push_back(_point);
		}
	}

	void VLP16PointCloud::transform(EulerTransformParameters &pose)
	{
	cu_.transformCloud(      p_raw_data_
			       , p_raw_transformed_data_

			       ,pose.x//, 0.           // x translate
			       ,pose.y//, 0.           // y translate
			       ,pose.z//, 1.5          // z translate
			       ,pose.theta_x//, -2 * PI/180  // roll
			       ,pose.theta_y//, 22 * PI/180  // pith
			       ,pose.theta_z//, 0.           // supposedly, yaw..?
			      );

	transform_ = pose;

	}

	void VLP16PointCloud::rangeSelectCloud(
		     std::vector<int8_t> & ring_no
		   , float height
		   , float angle_min
		   , float angle_max
		  )
	{
	  


	}
}
