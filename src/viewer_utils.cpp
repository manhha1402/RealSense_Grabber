#include "viewer_utils.h"

pcl::PointCloud<pcl::PointXYZRGB> viewer_utils::realsense_get_point_cloud()
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = getWidth() * getHeight();
    cloud.height = 1;
    cloud.resize( cloud.width*cloud.height);
    // (3) Acquire color and depth data
        const uint16_t* depth_data_aligned = getDepthDataPtr();
        const uint8_t* color_data = getColorDataPtr();
        rs2_intrinsics color_K_clone = color_intrin();
  float color_point[3], scaled_depth;
  for(int y=0; y<getHeight(); ++y)
  {
      for(int x=0; x<getWidth(); ++x)
      {
        pcl::PointXYZRGB pt;
        //get depth value at pixel [x,y]
        uint16_t depth_value = depth_data_aligned[y*getWidth()+x];
        //Get actual depth
        scaled_depth = depth_value *depthValue();
        //if no information, continue
        //if(scaled_depth==0) continue;
        //Get x,y,z from color coordinate
        float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        rs2_deproject_pixel_to_point(color_point, &color_K_clone, color_pixel, scaled_depth);

        pt.x = color_point[0];
        pt.y = color_point[1];
        pt.z = color_point[2];
        auto i = static_cast<int>(color_pixel[0]);
        auto j = static_cast<int>(color_pixel[1]);
        auto offset = i * 3 + j * getWidth() * 3;
        pt.r = static_cast<uint8_t>(color_data[offset]);
        pt.g = static_cast<uint8_t>(color_data[offset + 1]);
        pt.b = static_cast<uint8_t>(color_data[offset + 2]);
        cloud.push_back(pt);
        }
  }
 return cloud;
}

cv::Mat viewer_utils::getColorMat()
{
    cv::Mat color_image(getWidth(),getHeight(),CV_8UC3,(void*)getColorDataPtr(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_image, color_image, CV_BGR2RGB);
    return color_image;
}
cv::Mat viewer_utils::getDepthMat()
{
    cv::Mat depth_image(getWidth(),getHeight(),CV_16U,(void*)getDepthDataPtr(),cv::Mat::AUTO_STEP);
    return depth_image;
}

/*
std::shared_ptr<pcl::visualization::PCLVisualizer>
    visualize_rgb_pc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud)
{
    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0,"reference");
    viewer->setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
    viewer->initCameraParameters();
    return viewer;
}
void loop_viewer(const std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
    }
}

}
*/
