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
    void initViewer();
    void keyboardCallback (const pcl::visualization::KeyboardEvent &event);

private:
    pcl::visualization::PCLVisualizer viewer_;
};
