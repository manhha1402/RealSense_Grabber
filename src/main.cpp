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
//#include <OpenNI.h>
//#include "viewer_utils.h"
using namespace std;

void initViewer(pcl::visualization::PCLVisualizer &viewer);
void keyboardCallback (const pcl::visualization::KeyboardEvent &event);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
cv::Mat color_image, depth_image;
int frames_saved = 0;

int main() try
{
    realsense dev;
    //  Open Viewer
    pcl::visualization::PCLVisualizer viewer("Point Cloud");
    initViewer(viewer);
    viewer.addPointCloud(cloud);
    //Iterate
        while(!viewer.wasStopped()) {
            // Acquire new frame
            dev.getData();
            // (3-2) Visualize cloud , depth and color image
            color_image = dev.colorImage();
            depth_image = dev.depthImage();
            cv::imshow("Color Stream", color_image);
            cv::imshow("Depth Stream",depth_image);
            *cloud = dev.getPointCloud();
            viewer.updatePointCloud(cloud);
            viewer.spinOnce();
        }

  //  dev.init();

   // dev.printInformation();
    cout<<"helo"<<endl;

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
    std::string cloud_file;
    std::string color_image_file, depth_image_file;
    if(event.getKeySym() == "s" && event.keyUp()) {
        std::cout << "Saving frame " << frames_saved << "...\n";
        out << frames_saved;
        cloud_file = "cloud" + out.str() + ".pcd";
        color_image_file = "color" + out.str() +".png";
        depth_image_file = "depth" + out.str() + ".png";
        pcl::io::savePCDFile(cloud_file, *cloud);
        cv::imwrite(color_image_file,color_image);
        cv::imwrite(depth_image_file,depth_image);
        std::cout << "saved " <<frames_saved<< std::endl;
                frames_saved++;
    }
}
