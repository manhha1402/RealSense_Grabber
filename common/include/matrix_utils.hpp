#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <sstream>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
using namespace std;

void writeTransforms(std::string file_name, std::vector<Eigen::Matrix4f> transforms,std::vector<int> pose_flags);
void readTransforms(std::string file_name,std::vector<Eigen::Matrix4f>& transforms,std::vector<int>& pose_flags,int data_n);
// Constructs a plane from a collection of points
// so that the summed squared distance to all points is minimzized
Eigen::Vector3f leastSquare3d(const std::vector<Eigen::Matrix4f>& transforms,const std::vector<int>& mask);
Eigen::Vector3f projectPointToPlane(Eigen::Vector3f p,Eigen::Vector3f x);
Eigen::Matrix4f projectTransformToPlane(Eigen::Matrix4f t,Eigen::Vector3f x);
void projectTransformTranslationsToPlane(Eigen::Vector3f x, std::vector<Eigen::Matrix4f>& transforms, std::vector<int> mask);
