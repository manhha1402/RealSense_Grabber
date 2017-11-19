
// 
// Utilities for realsense usage
// 
#pragma once

// Personal libs
#include "realsense.h"

// Librealsense
#include <librealsense2/rs.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rs_utils
{

// 
// Generates a PCL point cloud from the depth and color image of a realsense camera
// 
 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const uint16_t*& depth_data_aligned,const uint8_t*& color_data
                                                       ,const double scale);
// 
// Generates a PCL point cloud from the depth and color image of a realsense camera
// 
// @param dev Realsense device wrapper
// @return Point cloud based on the realsense camera information
// 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const realsense& dev);

}
