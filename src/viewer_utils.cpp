#include "viewer_utils.h"

void viewer_utils::initViewer() {

    viewer_("Point Cloud");
    viewer_.setBackgroundColor(0, 0, 0);
    viewer_.addCoordinateSystem(1.0);
    viewer_.initCameraParameters();
    viewer_.setRepresentationToPointsForAllActors();
    viewer_.setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
    viewer_.registerKeyboardCallback(keyboardCallback);
}

void viewer_utils::keyboardCallback (const pcl::visualization::KeyboardEvent &event)
{
    int frames_saved = 0;
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
