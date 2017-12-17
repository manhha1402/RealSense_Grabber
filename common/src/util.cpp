#include "util.hpp"
#define X_LOWER_BOUND          0.09
#define X_UPPER_BOUND           0.25
#define Y_LOWER_BOUND           0.05
#define Y_UPPER_BOUND           0.2
#define Z_LOWER_BOUND           0.007
#define Z_UPPER_BOUND           0.4
void Utils::save2ply (pcl::PointCloud<pcl::PointXYZRGB> cloud,const std::string name)
{
	int num = cloud.points.size ();
	  std::ofstream myfile;
	  myfile.open(name.c_str());
	  myfile << "ply" << std::endl;
	  myfile << "format ascii 1.0" << std::endl;
	  myfile << "comment manh.ha.hoang@ipa.fraunhofer.de" << std::endl;
	  myfile << "element vertex " << num << std::endl;
	  myfile << "property float x" << std::endl;
	  myfile << "property float y" << std::endl;
	  myfile << "property float z" << std::endl;
	  myfile << "property uchar red" << std::endl;
	  myfile << "property uchar green" << std::endl;
	  myfile << "property uchar blue" << std::endl;
	  myfile << "end_header" << std::endl;

	  for (int i = 0; i < num; i++)
	  {
	    myfile << float (cloud.points[i].x) << " " << float (cloud.points[i].y)
	        << " " << float (cloud.points[i].z) << " ";
	    myfile << int (cloud.points[i].r) << " " << int (cloud.points[i].g)
	        << " " << int (cloud.points[i].b) << " " << std::endl;
	  }
	myfile.close ();
}
void Utils::test()
{
    std::cout<<camera_matrix_<<std::endl;
}

void Utils::drawPose(const cv::Mat& rot, const cv::Mat& trans, const cv::Mat& image,const pcl::PointXYZRGB& min_point,
                    const pcl::PointXYZRGB& max_point)
{

	std::vector<cv::Point2f> cube_imagePoints;
		cv::Mat img = image.clone();

		float length = 0.1;
		vector<cv::Point3f> axisPoints;
		axisPoints.push_back(cv::Point3f(0,0,0));
		axisPoints.push_back(cv::Point3f(length,0,0));
		axisPoints.push_back(cv::Point3f(0,length,0));
		axisPoints.push_back(cv::Point3f(0,0,length));
		vector<cv::Point2f> imagePoints;

        cv::projectPoints(axisPoints,rot,trans,camera_matrix_,cv::Mat(),imagePoints);
		// draw axis lines
		cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
		cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
		cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);



		std::vector<cv::Point3f> objectPoints; //coordinates of tag system
        objectPoints.push_back(cv::Point3f(max_point.x,max_point.y,0)); //C -0
        objectPoints.push_back(cv::Point3f(max_point.x,min_point.y,0)); //B-1
        objectPoints.push_back(cv::Point3f(min_point.x,max_point.y,0)); //D-2
        objectPoints.push_back(cv::Point3f(min_point.x,min_point.y,0));//A-3

        objectPoints.push_back(cv::Point3f(max_point.x,max_point.y,max_point.z)); //C1 -0
        objectPoints.push_back(cv::Point3f(max_point.x,min_point.y,max_point.z)); //B1 - 1
        objectPoints.push_back(cv::Point3f(min_point.x,max_point.y,max_point.z)); //D1-2
        objectPoints.push_back(cv::Point3f(min_point.x,min_point.y,max_point.z)); //A1 -3


	 //Project objectPoints into image coordinates
            cv::projectPoints( objectPoints,rot,trans, camera_matrix_,cv::Mat(),cube_imagePoints);
	        cv::line(img,cube_imagePoints[0],cube_imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);//0-1
	        cv::line(img,cube_imagePoints[0],cube_imagePoints[2],cv::Scalar(0,0,255,255),1,CV_AA);//0-2
	        cv::line(img,cube_imagePoints[3],cube_imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);
	        cv::line(img,cube_imagePoints[3],cube_imagePoints[2],cv::Scalar(0,0,255,255),1,CV_AA);

	        cv::line(img,cube_imagePoints[4],cube_imagePoints[5],cv::Scalar(0,0,255,255),1,CV_AA);
	        cv::line(img,cube_imagePoints[4],cube_imagePoints[6],cv::Scalar(0,0,255,255),1,CV_AA);
	        cv::line(img,cube_imagePoints[7],cube_imagePoints[5],cv::Scalar(0,0,255,255),1,CV_AA);
	        cv::line(img,cube_imagePoints[7],cube_imagePoints[6],cv::Scalar(0,0,255,255),1,CV_AA);
	        for (int i=0;i<4;i++)
	        	{
	        	    cv::line(img,cube_imagePoints[i],cube_imagePoints[i+4],cv::Scalar(0,0,255,255),1,CV_AA);
	        	}
		cv::imshow("pose",img);
		cv::waitKey(0);
}

Utils::Utils()
{
	viewer_ = (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_->setSize (480, 640);
	viewer_->setPosition (480, 200);

}
Utils::~Utils()
{

}
void Utils::viewCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{

    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer_->addPointCloud(cloud,rgb, "output");
    viewer_->spinOnce();


}
void Utils::viewMesh(const pcl::PolygonMesh& mesh)
{
    viewer_->removeAllPointClouds();
    viewer_->addPolygonMesh(mesh,"meshes",0);
    viewer_->spinOnce();

}

void Utils::visual(Eigen::Matrix4f m,pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
	viewer_->removeAllPointClouds();
	viewer_->addPointCloud(cloud.makeShared());
	Eigen::Matrix3f r;
	r<< m(0, 0), m(0, 1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2);
	Eigen::Vector3f t(m(0,3),m(1,3),m(2,3));
	Eigen::Affine3f pose = viewer_->getViewerPose();
	pose.linear()=r;
	pose.translation()=t;
	Eigen::Vector3f pos_vector = pose * Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = pose.rotation () * Eigen::Vector3f (0, 1, 0);
	viewer_->setCameraPosition(pos_vector[0],pos_vector[1],pos_vector[2],look_at_vector[0],look_at_vector[1],look_at_vector[2],
	up_vector[0],up_vector[1],up_vector[2]);
	viewer_->spinOnce();
}
void Utils::bbox3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
   /*Compute the 3x3 covariance matrix of a given set of points.
   * The result is returned as a Eigen::Matrix3f.
   * Note: the covariance matrix is not normalized with the number of points
   */
    pcl::PointXYZRGB min_point,max_point;
   Eigen::Vector4f pcaCentroid;
   pcl::compute3DCentroid(*cloud,pcaCentroid);
   Eigen::Matrix3f covariance;
   pcl::computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
   //Compute eigenvectors and eigenvalues of covariance matrix
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
   //Eigen vectors
   Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
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
   pcl::getMinMax3D(*cloudPointsProjected, min_point, max_point);
   const Eigen::Vector3f meanDiagonal = 0.5f*(max_point.getVector3fMap() + min_point.getVector3fMap());
//Finally, the quaternion is calculated using the eigenvectors (which determines how the final box gets rotated),
//and the transform to put the box in correct location is calculated.
//The minimum and maximum points are used to determine the box width, height, and depth.
   const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations
   const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
   viewer_->addCube(bboxTransform, bboxQuaternion, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);
   std::cout<<"Size of object in x,y,z directions(meters): "<<std::endl
   << max_point.x - min_point.x <<std::endl << max_point.y - min_point.y <<std::endl<< max_point.z - min_point.z <<std::endl;
   std::stringstream ss;
   ss<<"x: "<< (max_point.x - min_point.x) <<" y: "<<(max_point.y - min_point.y)<<" z: "<<(max_point.z - min_point.z);
   viewer_->addText(ss.str(), 320, 240, 4.0, 0, 0, "text");

}
void Utils::closeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
     //closes the cloud by adding a plate at the bottom
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate(new pcl::PointCloud<pcl::PointXYZRGB>);
    //get the ring of points at the bottom of the cloud
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (Z_LOWER_BOUND,Z_LOWER_BOUND+0.05);
    pass.filter (*plate);



    //take slices of the ring and add points to fill in the cloud
    for(float x=X_LOWER_BOUND;x <= X_UPPER_BOUND;x+=0.001)
    {
        pcl::PointCloud<pcl::PointXYZRGB> strip;
        pass.setInputCloud(plate);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x, x+.01); //slide size
        pass.filter(strip);
        std::cout<<strip.points.size()<<std::endl;

        if(strip.size()==0) {continue;} //no points, dont do anything

        //find the min and max y values of this strip
        float min = 9;
        float max = -9;
        for(int i = 0; i<strip.size(); i++)
        {
            float y = strip.points[i].y;
            if(y<min)
                  min = y;
             if(y>max)
                  max = y;
        }
        //add points from ymin to ymax for this strip
        for(float y = min+.01; y<=max-.01; y+=.001){
                  pcl::PointXYZRGB p;
                  p.x = x;
                  p.y = y;
                  p.z = Z_LOWER_BOUND;
                  //point white color
                  p.r = 255;
                  p.g = 255;
                  p.b = 255;
                  cloud->push_back(p);

         }
    }

}

void Utils::isStopped()
{
    while (!viewer_->wasStopped ())
		   {

			viewer_->spin ();
		     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		  }

}
void Utils::visualMatching(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoint_src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		keyPoint_tgt,pcl::CorrespondencesPtr correspondences)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousCloudTrans(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousKeypointsTrans(new pcl::PointCloud<pcl::PointXYZRGB>);
		   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		   viewer.reset(new pcl::visualization::PCLVisualizer);

		   pcl::transformPointCloud(*cloud_tgt, *previousCloudTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		   pcl::transformPointCloud(*keyPoint_tgt, *previousKeypointsTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		   cout << "previous keypoints: " << previousKeypointsTrans->size() << endl;
		   cout << "current keypoints: " << keyPoint_src->size() <<endl;
		   cout << "corres: " << correspondences->size() << endl;

		   viewer->addPointCloud(cloud_src, "current cloud");
		   viewer->addPointCloud(previousCloudTrans, "previous cloud");

		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_handler1(keyPoint_src, 0, 0, 255);
		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_handler2(keyPoint_tgt, 255, 0, 0);

		  viewer->addPointCloud(keyPoint_src, keypoints_handler1, "currentkeypoints");
		  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "currentkeypoints");

		  viewer->addPointCloud(previousKeypointsTrans, keypoints_handler2, "previouskeypoints");
		    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "previouskeypoints");


		    for (size_t i = 0; i < correspondences->size(); i++) {
		        stringstream line_ID;
		        line_ID << "correspondence_line_" << i;


		       viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZRGB>(keyPoint_src->at(correspondences->at(i).index_query),
		                                           previousKeypointsTrans->at(correspondences->at(i).index_match), 0, 255, 0, line_ID.str());
		      }

		   while (!viewer->wasStopped ())
		   	   	          {
		              viewer->spinOnce ();
		   	   	       }
}




















