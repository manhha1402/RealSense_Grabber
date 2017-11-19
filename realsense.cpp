#include "realsense.h"

realsense::realsense()
{
    //Enable color and depth streams with standard configuration
    config_.enable_stream(RS2_STREAM_DEPTH);
    config_.enable_stream(RS2_STREAM_COLOR,640,480, RS2_FORMAT_RGB8, 30);
    //
    selection_ = pipe_.start(config_);
    depth_scale_ = selection_.get_device().first<rs2::depth_sensor>().get_depth_scale();

}
realsense::~realsense()
{
}
void realsense::printInformation()
{
     auto color_stream = selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
     auto depth_stream = selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
     rs2_intrinsics color_K = color_stream.get_intrinsics();
     rs2_intrinsics depth_K = depth_stream.get_intrinsics();
     rs2_extrinsics depth2color_ext = depth_stream.get_extrinsics_to(color_stream);
     ///// Color Information ////
    auto principal_point = std::make_pair(color_K.ppx, color_K.ppy);
    auto focal_length = std::make_pair(color_K.fx, color_K.fy);
    rs2_distortion model = color_K.model;
    std::cout <<"Color sensor infomation "<<std::endl;
    std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
    std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
    std::cout << "Distortion Model        : " << model << std::endl;
    std::cout << "Distortion Coefficients : [" << color_K.coeffs[0] << "," << color_K.coeffs[1] << ","
    << color_K.coeffs[2] << "," << color_K.coeffs[3] << "," << color_K.coeffs[4] << "]" << std::endl;
    ///////// Depth Information ////////////
    auto principal_point_depth = std::make_pair(depth_K.ppx, depth_K.ppy);
    auto focal_length_depth = std::make_pair(depth_K.fx, depth_K.fy);
    rs2_distortion model_depth = depth_K.model;
    std::cout <<"Depth sensor infomation "<<std::endl;
    std::cout << "Principal Point         : " << principal_point_depth.first << ", " << principal_point_depth.second << std::endl;
    std::cout << "Focal Length            : " << focal_length_depth.first << ", " << focal_length_depth.second << std::endl;
    std::cout << "Distortion Model        : " << model_depth << std::endl;
    std::cout << "Distortion Coefficients : [" << depth_K.coeffs[0] << "," << depth_K.coeffs[1] << ","
    << depth_K.coeffs[2] << "," << depth_K.coeffs[3] << "," << depth_K.coeffs[4] << "]" << std::endl;
    //  camera-to-camera extrinsics from depth sensor to color sensor
    std::cout <<" extrinsics from depth sensor to color sensor "<<std::endl;
    std::cout<<"["<<depth2color_ext.rotation[0] <<" "<<depth2color_ext.rotation[1]<< " "<<depth2color_ext.rotation[2]<<std::endl;
    std::cout<<depth2color_ext.rotation[3] <<" "<<depth2color_ext.rotation[4]<< " "<<depth2color_ext.rotation[5]<<std::endl;
    std::cout<<depth2color_ext.rotation[6] <<" "<<depth2color_ext.rotation[7]<< " "<<depth2color_ext.rotation[8]<<"]"<<std::endl;
    //Depth Scale
    std::cout<<"Depth Scale :"<<std::endl<< depth_scale<<std::endl;
}
rs2::vertex* realsense::getVertices() const
{
    // Using the align object, we block the application until a frameset is available
    while (!frameset_.first_or_default(RS2_STREAM_DEPTH) || !frameset_.first_or_default(align_to))
    {
    frameset_ = pipe_.wait_for_frames();
    }
    //Align points from depth sensor to color sensor
    rs2::align align(align_to);
    auto proccessed = align.proccess(frameset_);
    // Trying to get both color and aligned depth frames
    auto color_frame = proccessed.get_color_frame();
    auto aligned_depth_frame = proccessed.get_depth_frame();
    //Get points by calculating depth information
    points_ =pc_.calculate(aligned_depth_frame);
    // Tell pointcloud object to map to this color frame
    pc_.map_to(color_frame);


}
