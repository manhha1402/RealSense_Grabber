#pragma once


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <iostream>



class Segmentation {

public:
	//void cloud2image(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,const pcl::PointIndices::Ptr& inliers);
	//void createMask(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,cv::Mat& mask);
	void euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min, int max, double tolerance,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& output);

private:



};

