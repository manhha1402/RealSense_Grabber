#include "realsense.h"

realsense::realsense()
{
    std::cout<<"hello"<<std::endl;
    //Enable color and depth streams with standard configuration
    config_.enable_stream(RS2_STREAM_DEPTH);
    config_.enable_stream(RS2_STREAM_COLOR,640,480, RS2_FORMAT_RGB8, 30);
    //Start device, get neccessary information
    selection_ = pipe_.start(config_);
    //Depth scale
    depth_scale_ = selection_.get_device().first<rs2::depth_sensor>().get_depth_scale();
    //Intrinsic parameters
    color_K = selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    depth_K = selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    //Extrinsic depth to color
    depth2color_ext = selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_extrinsics_to(
                selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>());
}
realsense::~realsense()
{
}
bool realsense::isConnected() const
{
    auto list = ctx_.query_devices();
    return list.size()>0;
}
void realsense::printInformation()
{
  if(isConnected())
   {
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
    std::cout<<"Depth Scale :"<<std::endl<< depth_scale_<<std::endl;
  }
  else
  {
          throw std::runtime_error("Cannot get device information. No device connected.");
  }
}
rs2_intrinsics realsense::color_intrin() const
{
    return color_K;
}
rs2_intrinsics realsense::depth_intrin() const
{
    return depth_K;
}

void realsense::wait_for_frames() const
{
     rs2::frameset frameset_;
    // Using the align object, we block the application until a frameset is available
    while (!frameset_.first_or_default(RS2_STREAM_DEPTH) || !frameset_.first_or_default(align_to))
    {
    frameset_ = pipe_.wait_for_frames();
    }
}
const uint16_t* realsense::getDepth()
{
    if(!isConnected())
    {
        throw std::runtime_error("Cannot get depth image. No device connected.");
    }
    rs2::frameset frameset_;
    frameset_ = pipe_.wait_for_frames();
    //Align points from depth sensor to color sensor
    rs2::align align(align_to);
    auto proccessed = align.proccess(frameset_);
    // Trying to get aligned depth frames
    rs2::depth_frame aligned_depth_frame = proccessed.get_depth_frame();
    //Retrieve aligned depth data
     const uint16_t* depth_data_aligned = reinterpret_cast<const uint16_t *>(aligned_depth_frame.get_data());
 //   return std::vector<uint16_t>(depth_data_aligned, depth_data_aligned + (sizeof(depth_data_aligned) / sizeof(depth_data_aligned[0])));
    return depth_data_aligned;
}
const uint8_t* realsense::getColor()
{

    if(!isConnected())
    {
        throw std::runtime_error("Cannot get color image. No device connected.");
    }
    rs2::frameset frameset_;
    frameset_ = pipe_.wait_for_frames();
    // Trying to get color frames
    rs2::video_frame color_frame = frameset_.get_color_frame();
    //Retrieve color data
    const uint8_t * color_data = reinterpret_cast<const uint8_t *>(color_frame.get_data());
    return color_data;
}
