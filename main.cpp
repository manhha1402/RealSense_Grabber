#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/common/common.h>
#include "realsense.h"
using namespace std;
void view(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    viewer.reset(new pcl::visualization::PCLVisualizer);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
          viewer->addPointCloud(cloud,rgb, "output");

           while (!viewer->wasStopped ())
              {
                viewer->spinOnce ();
              }
}
int main() try
{

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::io::loadPCDFile("../meanregistration_cloud.pcd",*cloud);
    realsense dev;
    /*Compute the 3x3 covariance matrix of a given set of points.
   * The result is returned as a Eigen::Matrix3f.
   * Note: the covariance matrix is not normalized with the number ofpoints
   */
   /*
   Eigen::Vector4f pcaCentroid;
   pcl::compute3DCentroid(*cloud,pcaCentroid);
   Eigen::Matrix3f covariance;
   pcl::computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
   //Compute eigenvectors and eigenvalues of covariance matrix
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
   //Eigen vectors
   Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
   cout<<eigenVectorsPCA<<endl;
   /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
/// the signs are different and the box doesn't get correctly oriented in some cases.
   eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
   // Transform the original cloud to the origin point where the principal components correspond to the axes.
   Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
   projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
   projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::transformPointCloud(*cloud,*cloudPointsProjected,projectionTransform);
   // Get the minimum and maximum points of the transformed cloud.
   pcl::PointXYZRGB minPoint, maxPoint;
pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
//Finally, the quaternion is calculated using the eigenvectors (which determines how the final box gets rotated),
//and the transform to put the box in correct location is calculated.
//The minimum and maximum points are used to determine the box width, height, and depth.
const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations
const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

cout<<"pcaCentroid"<<endl<<pcaCentroid<<endl;
   cout<<"covariance"<<endl<<covariance<<endl;
   cout<<"eigenVectorsPCA"<<endl<<eigenVectorsPCA<<endl;
   cout<<"bboxTransform"<<endl<<bboxTransform<<endl;
   //cout<<"bboxQuaternion"<<endl<<bboxQuaternion<<endl;

   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
   viewer.reset(new pcl::visualization::PCLVisualizer);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    viewer->addPointCloud(cloud,rgb, "output");
 viewer->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);

          while (!viewer->wasStopped ())
             {
               viewer->spinOnce ();
             }

*/
    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
