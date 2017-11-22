#include "realsense_utils.h"
#include <librealsense2/rsutil.h>
namespace rs_utils
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const std::vector<uint16_t>& depth_data_aligned,const std::vector<uint8_t>& color_data,
                                                       const rs2_intrinsics& color_K,const rs2_intrinsics& depth_K,const float depth_scale)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   float color_point[3], scaled_depth;
  for(int y=0; y<depth_K.height; ++y)
  {
      for(int x=0; x<depth_K.width; ++x)
      {
        pcl::PointXYZRGB pt;
        uint16_t depth_value = depth_data_aligned[y*depth_K.width+x];
        scaled_depth = depth_value *depth_scale;
        if(scaled_depth==0) continue;
        float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        rs2_deproject_pixel_to_point(color_point, &color_K, color_pixel, scaled_depth);
        pt.x = color_point[0];
        pt.y = color_point[1];
        pt.z = color_point[2];
        auto i = static_cast<int>(color_pixel[0]);
        auto j = static_cast<int>(color_pixel[1]);
        auto offset = i * 3 + j * color_K.width * 3;
        pt.r = static_cast<uint8_t>(color_data[offset]);
        pt.g = static_cast<uint8_t>(color_data[offset + 1]);
        pt.b = static_cast<uint8_t>(color_data[offset + 2]);
        cloud->points.push_back(pt);
        }
  }
 return cloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud (const realsense& dev)
{
    return get_point_cloud(dev.getDepth(),dev.getColor(),dev.color_intrin(),dev.depth_intrin(),dev.depthValue());
}
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
