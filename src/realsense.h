#ifndef REALSENSE_H
#define REALSENSE_H
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

static const rs2_stream align_to = RS2_STREAM_COLOR;
class realsense
{
public:
    realsense();
    ~realsense();

    void printInformation();  
    void getData();
    /*Getter*/
    inline bool isConnected() const {return ctx_.query_devices().size()>0 ;}
    // Get color intrinsic
    inline rs2_intrinsics color_intrin() const {return color_K;}
    //Get depth intrinsic
    inline rs2_intrinsics depth_intrin() const {return depth_K;}
    //Get depth scale
    inline float depthValue() const {return depth_scale_;}
    //Get height
    inline int getHeight() const {return height_;}
    //Get width
   inline int getWidth() const {return width_;}
    //Get size
   inline int getSize() const {return size_;}
    //Get depth image
     cv::Mat depthImage();
   // { return reinterpret_cast<const uint16_t*> (proccessed_.get_depth_frame().get_data()); }
    //Get color image
    cv::Mat colorImage();
  //{  return reinterpret_cast<const uint8_t*> (proccessed_.get_color_frame().get_data()); }
    pcl::PointCloud<pcl::PointXYZRGB> getPointCloud();
private:

    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::context ctx_;
    rs2::pipeline_profile selection_;
    rs2_intrinsics color_K;
    rs2_intrinsics depth_K;
    rs2_extrinsics depth2color_ext;
    float depth_scale_;
    rs2::frameset frameset_;
    rs2::frameset proccessed_;
    int height_,width_,size_;
};

#endif //REALSENSE_H
