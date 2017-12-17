#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/poisson.h>


/**
 * This class includes various methods for filtering the point cloud
 *
 */
class Filters{

public:

    /** Use a PassThrough filter to remove points with depth values that are too large or too small */
    void
    threshold(pcl::PointCloud<pcl::PointXYZRGB>& input,const char* axis, float min_value, float max_value);

/** Use a VoxelGrid filter to reduce the number of points */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size);
/** Use a StatisticalOutlierRemoval filter to remove all points with too few local neighbors */
    void
    removeOutliers (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors);
/** remove all NaN values from Normals */
    pcl::PointCloud<pcl::PointNormal>::Ptr
    removeNaNNormals (const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormal, const std::string filename);
/*  MovingLeastSquares represent an implementation of the MLS (Moving Least Squares) algorithm
    * for data smoothing and improved normal estimation. It also contains methods for upsampling the
    * resulting cloud based on the parametric fit.
    * Output has the PointNormal type in order to store the normals calculated by MLS
    *
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    movingLeastSquares(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,float radius);
/*
 * Create mesh of point clouds using poisson
 */
    pcl::PolygonMesh surfaceReconstruction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
private:

};
