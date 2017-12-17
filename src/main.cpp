#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>

//Header files
#include "realsense.h"
#include "util.hpp"
#include "registrator.hpp"
#include "filters.hpp"
#include "segmentation.hpp"
#include "loader.hpp"
#include "fiducial/FiducialDefines.h"
#include "fiducial/FiducialModelPi.h"
#include "fiducial/AbstractFiducialModel.h"
#include "RobustMatcher.h"
using namespace std;
using namespace ipa_Fiducials;
void initViewer(pcl::visualization::PCLVisualizer &viewer);
void keyboardCallback (const pcl::visualization::KeyboardEvent &event);
void keyboardCallbackRoi(int event, int x,int y, int flags, void* param);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
cv::Mat color_image, depth_image;
int frames_saved = 0;
bool clicked = false;

cv::Mat camera_matrix =  (cv::Mat_<double>(3,3)<<   619.69,0,313.013 ,
                                                 0,619.69, 245.14,
0,0,1);
int main(int argc,char** argv) try
{
    //std::string model_filename = argv[1]; //pi tag file
    cv::Mat img = cv::imread("../frame-000002.color.png",1);
    RobustMatcher rmatcher;
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create(400);
    rmatcher.setFeatureDetector(detector);
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;
    rmatcher.computeKeyPoints(img,keypoints);
    cv::drawKeypoints(img,keypoints,img,cv::Scalar(-1));
    cv::imshow("sad",img);
    cv::waitKey(0);
    //cout<<keypoints.size()<<endl;



    /*
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
            cout<<"Distance from color camera to center of image "<<dev.getCenterDistance()<<endl;
            viewer.updatePointCloud(cloud);
            viewer.spinOnce();
        }

  //  dev.init();

   // dev.printInformation();
    cout<<"helo"<<endl;
*/
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
