#ifndef __UTIL_
#define __UTIL_
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include "config.hpp"
using namespace std;

extern double minX;
extern double maxX;
extern double minY;
extern double maxY;
extern double minZ;
extern double maxZ; /*Cropped Axis*/


class Utils : public Config{
public:
	Utils();
	~Utils();
    void test();
	//save cloud to .ply file
	void save2ply (pcl::PointCloud<pcl::PointXYZRGB> cloud,const std::string name);
	//Draw marker pose
    void drawPose(const cv::Mat& rot, const cv::Mat& trans, const cv::Mat& image, const pcl::PointXYZRGB &min_point, const pcl::PointXYZRGB &max_point);
	//Simple function to view point cloud
    void viewCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	//Function to visulize the point cloud registration from camera pose
	void visual(Eigen::Matrix4f m,pcl::PointCloud<pcl::PointXYZRGB> cloud);
        //Function to viluasize final cloud
    void isStopped();
	//Function to visualize key point matchings between 2 clouds
	void visualMatching(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoint_src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                    keyPoint_tgt,pcl::CorrespondencesPtr correspondences);
        /*Draw bounding box of point cloud using PCA
        * 1. Compute mean(centroid) of point cloud
        * 2. Compute normalized covariance matrix (mean removal) of point cloud
        * 3. Compute eigenvectors and eigenvalues of covariance matrix (e0,e1,e2) -> Reference system(e0,e1,e0xe1)
        * 4. Move the points to RF and compute the max, the min and the center of the diagonal
        * 5. Compute Translation and Quaternion of points
        */
    void bbox3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void closeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void viewMesh(const pcl::PolygonMesh& mesh);
private:
	pcl::visualization::PCLVisualizer  *viewer_;


};

class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void
  copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
	  // < x, y, z, curvature >
	     out[0] = p.x;
	     out[1] = p.y;
	     out[2] = p.z;
	     out[3] = p.curvature;
  }
};


class MyPointRepresentation1 : public pcl::PointRepresentation<pcl::PointXYZRGB>
{
  using pcl::PointRepresentation<pcl::PointXYZRGB>::nr_dimensions_;
public:
  MyPointRepresentation1 ()
  {
    nr_dimensions_ = 6;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void
  copyToFloatArray (const pcl::PointXYZRGB &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    // RGB -> YUV
    out[3] = float (p.r) * 0.299 + float (p.g) * 0.587 + float (p.b) * 0.114;
    out[4] = float (p.r) * 0.595716 - float (p.g) * 0.274453 - float (p.b) * 0.321263;
    out[5] = float (p.r) * 0.211456 - float (p.g) * 0.522591 + float (p.b) * 0.311135;
  }
};
#endif //__UTIL_
