#include "realsense_utils.h"
namespace rs_utils
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const uint16_t*& depth_data_aligned,const uint8_t*& color_data,
                                                       const rs2_intrinsics& color_K,const rs2_intrinsics& depth_K,const double scale)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int y=0; y<depth_K.height; ++y)
      {
      for(int x=0; x<depth_K.width; ++x)

      {



}
}
