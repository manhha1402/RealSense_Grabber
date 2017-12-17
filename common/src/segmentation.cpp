#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <limits>
#include "../include/segmentation.hpp"
/*
void Segmentation::cloud2image(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,const pcl::PointIndices::Ptr& inliers)
{	
bool debug = true;

	cv::Mat temp_img;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clone (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_clone->width = img.cols;
	cloud_clone->height = img.rows;
	//cloud_clone->points.resize(cloud_clone->width * cloud_clone->height);
	int w = img.cols;
	int h = img.rows;
			cloud_clone->is_dense = true;
			 for(int x =0; x<cloud->width;x++)
							          for(int y =0;y<cloud->height;y++)
							          {
							            PointRGB& point = cloud->points[y*w+x];
							            Eigen::Vector3i rgb = point.getRGBVector3i();
							            if(rgb[2]!=0 && rgb[1] !=0 && rgb[0] !=0)
							            {
							            	 cloud_clone->points[y*w+x] = point;
							            }
							            else
							            {
			cloud_clone->points[y*w+x].x = cloud_clone->points[y*w+x].y =
					cloud_clone->points[y*w+x].z=std::numeric_limits<float>::quiet_NaN();
							            }
							          }




			temp_img = cv::Mat::zeros(img.size(),CV_8UC3);


		if(cloud->isOrganized()){
		  for (int h=0; h<img.rows; h++) {
		                for (int w=0; w<img.cols; w++) {
		                    PointRGB& point = cloud_clone->at(w, h);
		                    Eigen::Vector3i rgb = point.getRGBVector3i();
		                    temp_img.at<cv::Vec3b>(h,w)[0] = rgb[2];
		                    temp_img.at<cv::Vec3b>(h,w)[1] = rgb[1];
		                    temp_img.at<cv::Vec3b>(h,w)[2] = rgb[0];
		                      }
		                  }
		              }
		else
		{
			cout<<"cloud is organized"<<endl;
		}
		cout<<"check"<<endl;

		if(debug)
		{
			cv::imshow("img",img);
			cv::imshow("cloud2image",temp_img);
			cv::waitKey(0);
		}
}
void Segmentation::createMask(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,cv::Mat& mask)
{
	bool debug = false;//show segmented image

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clone (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_clone = *cloud;
		cloud_clone->is_dense = true;
	cv::Mat temp_img;
	temp_img = cv::Mat::zeros(img.size(),CV_8UC3);
	if(cloud_clone->isOrganized()){
	  for (int h=0; h<img.rows; h++) {
	                for (int w=0; w<img.cols; w++) {
	                    pcl::PointXYZRGB& point = cloud_clone->at(w, h);
	                    Eigen::Vector3i rgb = point.getRGBVector3i();
	                    temp_img.at<cv::Vec3b>(h,w)[0] = rgb[2];
	                    temp_img.at<cv::Vec3b>(h,w)[1] = rgb[1];
	                    temp_img.at<cv::Vec3b>(h,w)[2] = rgb[0];
	                      }
	                  }
	              }

	  cv::Mat black,segment_img;
	  black= cv::Mat::zeros(img.size(),CV_8UC3);
	  //create black image with 2D white bounding box inside that should contain objects
	  cv::rectangle(black,cv::Rect(x_box,y_box,width_box,height_box),cv::Scalar(255,255,255),-1,8,0);
	  //perform bitwise_and to remove outside area
	  cv::bitwise_and(temp_img,black,segment_img);
	  mask = cv::Mat::ones(img.size(),CV_32F)*255.;
	  for (int h=0; h<img.rows; h++)
	  {
		  for (int w=0; w<img.cols; w++)
	                 {
	                	 if((segment_img.at<cv::Vec3b>(h,w)[0] ==0) && (segment_img.at<cv::Vec3b>(h,w)[1] ==0) && (segment_img.at<cv::Vec3b>(h,w)[2] ==0))
	                	 {
	                		 mask.at<float>(h,w) = 0.;
	                	 }
	                 }
	  }

	   int dilation_size = 2;
	          int dilation_type = cv::MORPH_RECT;//MORPH_RECT MORPH_CROSS  MORPH_ELLIPSE;
	          cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
	           cv::Point( dilation_size, dilation_size ) );
	         // cv::dilate( mask, mask, element );
	          cv::morphologyEx(mask,mask,cv::MORPH_OPEN,element); //remove outliers
	          cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,element); //fill holes inside objects
	        //  cv::imwrite(mask_file,mask);
	if(debug)
	{
			cv::imshow("mask",mask);
		//	cv::imshow("segment_img",segment_img);
			//cv::imshow("temp_img",temp_img);
			cv::waitKey(0);
	}
}
*/
void Segmentation::euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min, int max, double tolerance,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& output)
{	
// Creating the KdTree object for the search method of the extraction
  	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	    tree->setInputCloud(cloud);
 std::vector<pcl::PointIndices> cluster_indices;
 pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
 ec.setClusterTolerance (tolerance);
 ec.setMinClusterSize (min);
 ec.setMaxClusterSize (max);
 ec.setSearchMethod (tree);
 ec.setInputCloud (cloud);
 ec.extract (cluster_indices);
 pcl::console::print_value ("%d", cluster_indices.size ()); pcl::console::print_info (" clusters]\n");

 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       cloud_cluster->points.push_back (cloud->points[*pit]); //*
     cloud_cluster->width = cloud_cluster->points.size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = false;
     pcl::console::print_value ("%d \n",cloud_cluster->size());
     output.push_back(cloud_cluster);
   }

}

















