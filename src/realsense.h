#ifndef REALSENSE_H
#define REALSENSE_H
#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <memory>
#include <vector>
static const rs2_stream align_to = RS2_STREAM_COLOR;
class realsense {
public:

    ~realsense();
    void init();
    void printInformation();  
    int getData();
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
    //Get aligned depth stream
    inline const uint16_t*  getDepthDataPtr() const {return reinterpret_cast<const uint16_t*> (depth_frame_ref_.get_data()); }
    //Get color information
    inline const uint8_t*  getColorDataPtr() const {return reinterpret_cast<const uint8_t*> (color_frame_ref_.get_data()); }

private:

    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::context ctx_;
    rs2::pipeline_profile selection_;
    rs2_intrinsics color_K;
    rs2_intrinsics depth_K;
    rs2_extrinsics depth2color_ext;
    float depth_scale_;
    rs2::video_frame color_frame_ref_;
    rs2::depth_frame depth_frame_ref_;
    int height_,width_,size_;
};

#endif //REALSENSE_H
