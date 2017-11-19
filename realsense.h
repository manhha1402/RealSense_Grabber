#ifndef REALSENSE_H
#define REALSENSE_H
#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <memory>
#include <vector>
static const rs2_stream align_to = RS2_STREAM_COLOR;
class realsense {
public:
    realsense();
    ~realsense();
    bool isConnected() const;
    void printInformation();
    void wait_for_frames() const;
    // Get color intrinsic
    rs2_intrinsics color_intrin() const;
    //Get depth intrinsic
    rs2_intrinsics depth_intrin() const;
    //Get aligned depth stream
    //std::vector<uint16_t> getDepth() const;
    const uint16_t* getDepth() ;
    //Get color information
    const uint8_t* getColor();
    //Debugging
  //  void logFile(const std::string& log_file) const;
private:

    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::context ctx_;
    rs2::pipeline_profile selection_;
    rs2_intrinsics color_K;
    rs2_intrinsics depth_K;
    rs2_extrinsics depth2color_ext;
    float depth_scale_;

};

#endif //REALSENSE_H
