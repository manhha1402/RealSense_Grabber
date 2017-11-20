#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/common/common.h>
#include "realsense.h"
#include "realsense_utils.h"
using namespace std;

int main() try
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    realsense dev;
    dev.printInformation();
   while(dev.isConnected())
   {
       dev.wait_for_frames();
        auto point_cloud = rs_utils::get_point_cloud(dev);
        // Visualize the point cloud
        auto viewer = rs_utils::visualize_rgb_pc(point_cloud);
        rs_utils::loop_viewer(viewer);
   }

    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
