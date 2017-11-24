#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include "realsense.h"
#include "viewer_utils.h"
using namespace std;

void initViewer(pcl::visualization::PCLVisualizer &viewer);
void keyboardCallback (const pcl::visualization::KeyboardEvent &event);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
int frames_saved = 0;

int main() try
{
    realsense dev;
    dev.printInformation();

    // (2) Open Viewer
    pcl::visualization::PCLVisualizer viewer("Point Cloud");

    initViewer(viewer);
    viewer.addPointCloud(cloud);
   while(!viewer.wasStopped())
   {
        dev.wait_for_frames();
        cloud = viewer_utils::realsense_get_point_cloud(dev);
        // Visualize the point cloud       
        viewer.updatePointCloud(cloud);
        viewer.spinOnce();
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
void initViewer(pcl::visualization::PCLVisualizer &viewer) {
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.setRepresentationToPointsForAllActors();
    viewer.setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
    viewer.registerKeyboardCallback(keyboardCallback);
}

void keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
    std::stringstream out;
    std::string name;
    if(event.getKeySym() == "s" && event.keyUp()) {
        std::cout << "Saving frame " << frames_saved << "...\n";
                out << frames_saved;
                name = "InputCloud" + out.str() + ".pcd";
        pcl::io::savePCDFile(name, *cloud);
        std::cout << "done" << std::endl;
                frames_saved++;
    }
}
