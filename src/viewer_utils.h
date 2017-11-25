//
// Utilities for camera usage
// 
#pragma once
// Personal libs
#include "realsense.h"
// Librealsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
//OpenNI
//#include <OpenNI.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//OpenCV
#include <opencv2/opencv.hpp>
class viewer_utils : public realsense
{
public:
// 
// Generates a PCL point cloud from the depth and color image of a depth camera
// 
 
pcl::PointCloud<pcl::PointXYZRGB> realsense_get_point_cloud();
// 
// Generates a PCL point cloud from the depth and color image of a realsense camera
// 

//Generate cv::Mat color and depth image of cameras
cv::Mat getColorMat();
cv::Mat getDepthMat();
//Renser point cloud
/*
std::shared_ptr<pcl::visualization::PCLVisualizer>
visualize_rgb_pc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud);
void loop_viewer(const std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
*/
};
