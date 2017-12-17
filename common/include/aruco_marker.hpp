#ifndef MARKER_H
#define MARKER_H
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <boost/shared_ptr.hpp>
#include "tinyxml.h"
using namespace std;
struct cha_params {
    int square_x;
    int square_y;
    float square_length;
    float marker_length;
    int dictionary_id;
    int border_bits;
    int margins;
    };
struct a_params {
    int markers_x;
    int markers_y;
    int marker_length;
    int marker_separation;
    int dictionary_id;
    int margins;
    int border_bits;
};
struct aruco_pose {
    cv::Mat rot;
    cv::Mat trans;
};

class Marker
{
public:
    Marker();
    ~Marker();
    void setCameraMatrix(const cv::Mat& camera_matrix) {camera_matrix_ = camera_matrix;}
    unsigned long LoadParameters(std::string directory_and_filename);
    unsigned long createChaRucoMarker(cv::Mat& board_image);
    unsigned long createArucoMarker(cv::Mat& board_images);
    bool estimatePoseCharuco(cv::Mat& frame,aruco_pose& pose);
private:
    cha_params charuco_params_;
    a_params aruco_params_;
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
    cv::Ptr<cv::aruco::Board> board_;
    cv::Mat camera_matrix_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};
#endif //MARKER_H
