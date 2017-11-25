#include "realsense.h"
#include <librealsense2/rs.hpp>
realsense::realsense() {
    std::cout<<"heelo"<<std::endl;

    //Set heigh,width,size
    height_ = 480; width_ = 640;size_ = height_*width_;
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

//Get streams of camera
void realsense::getData()
{
    //Check device opened

   if(!isConnected())
                {
                    throw std::runtime_error("Cannot get data. No device connected.");
                }
    //Get data

     rs2::align align(align_to);
    while (!frameset_.first_or_default(RS2_STREAM_DEPTH) || !frameset_.first_or_default(align_to))
     {
                frameset_ = pipe_.wait_for_frames();
    }
    //Align depth to color stream
     proccessed_ = align.proccess(frameset_);

}

 cv::Mat realsense::depthImage()
{
      rs2::depth_frame depth_frame=  proccessed_.get_depth_frame();
     cv::Mat depth_image = cv::Mat(cv::Size(getHeight(),getWidth()),CV_16UC1,(void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
     return depth_image;
}

 cv::Mat realsense::colorImage()
{
        rs2::frame color_frame = proccessed_.get_color_frame();
        cv::Mat color_image = cv::Mat(cv::Size(getHeight(),getWidth()),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
        cv::cvtColor(color_image,color_image,CV_BGR2RGB);
        return color_image;
    //return reinterpret_cast<const uint8_t*> (color_frame.get_data());
}
 pcl::PointCloud<pcl::PointXYZRGB> realsense::getPointCloud()
 {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.width = getWidth();
        cloud.height = getHeight();
        cloud.is_dense = false;
        const uint16_t * depth_data = reinterpret_cast<const uint16_t *> (proccessed_.get_depth_frame().get_data());
        const uint8_t * color_data = reinterpret_cast<const uint8_t *> (proccessed_.get_color_frame().get_data());
         float color_point[3], scaled_depth;
      for(int y=0;y<getHeight();y++)
        {
          for(int x=0;x<getWidth();x++)
          {
              pcl::PointXYZRGB pt;
              uint16_t depth_value = depth_data[getWidth()*y + x];
              scaled_depth = depth_value *depth_scale_;
              float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
              rs2_deproject_pixel_to_point(color_point, &color_K, color_pixel, scaled_depth);
              if (color_point[2] <= 0.f || color_point[2] > 5.f) continue;
              pt.x = color_point[0];
              pt.y = color_point[1];
              pt.z = color_point[2];
              auto i = static_cast<int>(color_pixel[0]);
              auto j = static_cast<int>(color_pixel[1]);

              auto offset = i * 3 + j * color_K.width * 3;
              pt.r = static_cast<uint8_t>(color_data[offset]);
              pt.g = static_cast<uint8_t>(color_data[offset + 1]);
              pt.b = static_cast<uint8_t>(color_data[offset + 2]);
              cloud.points.push_back(pt);

          }
        }
      return cloud;
 }

//Print some information of camera
void realsense::printInformation()
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

