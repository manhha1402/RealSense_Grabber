#ifndef REALSENSE_H
#define REALSENSE_H
#include <librealsense2/rs.hpp>
#include <iostream>
static const rs2_stream align_to = RS2_STREAM_COLOR;
class realsense {
public:
    realsense();
    ~realsense();
    void printInformation();
    //Get point cloud information
    rs2::vertex* getVertices() const;
    //Get color information

private:
    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::frameset frameset_;
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc_;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points_;
    rs2::pipeline_profile selection_;
    float depth_scale_;

};

#endif //REALSENSE_H
