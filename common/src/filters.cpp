#include "../include/filters.hpp"
#include <pcl/surface/poisson.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <iostream>
#include <limits>

void Filters::threshold(pcl::PointCloud<pcl::PointXYZRGB>& input,const char* axis, float min_value, float max_value) {
	   pcl::PointCloud<pcl::PointXYZRGB> tmp;
       pcl::PassThrough<pcl::PointXYZRGB> bbox_filter;
       bbox_filter.setFilterFieldName(axis);
       bbox_filter.setFilterLimits(min_value, max_value);
       bbox_filter.setInputCloud(input.makeShared());
       bbox_filter.filter(tmp);
       input = tmp;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (input);
    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter (*downsampled);
    return (downsampled);
}
void Filters::removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, float radius, int meanK) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    	 sor.setInputCloud (input);
    	 sor.setMeanK (meanK);
    	 sor.setStddevMulThresh (radius);
    	 sor.filter (*input);
}


pcl::PointCloud<pcl::PointNormal>::Ptr
Filters::removeNaNNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormal, const std::string filename) {
    pcl::console::print_highlight ("removing NaN values from normals...\n");

    pcl::PointCloud<pcl::PointNormal>::Ptr
            filteredNormal = boost::make_shared<pcl::PointCloud<pcl::PointNormal> >();
    pcl::PointCloud<pcl::PointNormal>::PointType p_nan;
    pcl::PointCloud<pcl::PointNormal>::PointType p_valid;

    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();

    filteredNormal->push_back(p_nan);

    p_valid.x = 1.0f;
    filteredNormal->push_back(p_valid);

    std::cout<<"previous size of "<< filename <<" is: " << inputNormal->points.size() << std::endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputNormal, *filteredNormal, indices);

    std::cout<<"filtered size of "<< filename <<" is: " << filteredNormal->points.size() << std::endl;

    return filteredNormal;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::movingLeastSquares(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,float radius)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	mls.setComputeNormals (true);
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (radius);
	mls.process (*mls_points);
	return mls_points;
}
pcl::PolygonMesh Filters::surfaceReconstruction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
   pcl::PolygonMesh mesh;
   pcl::PointCloud<pcl::PointXYZ>::Ptr surface (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::copyPointCloud(*cloud,*surface);
   pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne; //compute normals
   ne.setNumberOfThreads(8);
   ne.setInputCloud(surface);
   ne.setRadiusSearch(0.001);
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*surface,centroid);
   ne.setViewPoint(centroid[0],centroid[1],centroid[2]);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
   ne.compute(*cloud_normals);
   for(size_t i =0;i<cloud_normals->size();i++)
      {
        cloud_normals->points[i].normal_x *=-1;
        cloud_normals->points[i].normal_y *=-1;
        cloud_normals->points[i].normal_z *=-1;
     }
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smooth_normals(new  pcl::PointCloud<pcl::PointNormal>);
   pcl::concatenateFields(*surface,*cloud_normals,*cloud_smooth_normals);
   pcl::Poisson<pcl::PointNormal> poisson;
   poisson.setDepth(9);
   poisson.setInputCloud(cloud_smooth_normals);
   poisson.reconstruct(mesh);
   return mesh;
}
