#pragma once
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The Config class provides the API for the parameter configuration.It loads parameters from yaml file stored in "Config"
 * folder to perform needed functions
 *
 */
const std::string config_file_path = "../config/config.yaml";
class Config
{
protected:
    /*intrinsics matrix of camera*/
     cv::Mat camera_matrix_ ;
     float max_distance_;
public:


    Config()
    {

        // ********** try to load the yaml file that is located at the given path **********
        cv::FileStorage config_file(config_file_path,cv::FileStorage::READ);
        config_file["cameraMatrix"]>> camera_matrix_;
        config_file["maxDistance"]>>max_distance_;
    }
    ~Config()
    {
    }

// Getter funtions that return parameters from configuration file
      cv::Mat cameraMatrix() { return camera_matrix_;}
      float maxDistance() {return max_distance_;}
};
