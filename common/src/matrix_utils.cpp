#include "matrix_utils.hpp"

void writeTransforms(std::string file_name, std::vector<Eigen::Matrix4f> transforms,std::vector<int> pose_flags)
{
    cv::FileStorage fs(file_name,cv::FileStorage::WRITE);
    std::vector<cv::Mat> transforms_cv;
    for(int i=0;i<transforms.size();i++)
    {
        cv::Mat transform;
        cv::eigen2cv(transforms[i],transform);
        transforms_cv.push_back(transform);
    }
    std::string flag= "Flag";
    std::string transform_flag = "TRANSFORM";

    for(int i=0;i<transforms.size();i++)
    {
        std::ostringstream ss;
        ss<<i;
        std::string flag = "Flag " +ss.str();
        std::string transform_flag = "TRANSFORM " + ss.str();
        fs<<flag<<pose_flags[i];
        fs <<transform_flag<<transforms_cv[i];
    }


}
void readTransforms(std::string file_name,std::vector<Eigen::Matrix4f>& transforms,std::vector<int>& pose_flags,int data_n)
{
    cv::FileStorage fs(file_name,cv::FileStorage::READ);

    for(int i=0;i<data_n;i++)
    {
        std::ostringstream ss;
        ss<<i;
        cv::Mat matrix;
        Eigen::Matrix4f matrix_eigen;
        int pose_flag;
        std::string transform_flag = "TRANSFORM " + ss.str();
        std::string flag = "Flag " +ss.str();
        //Read transform
        fs[transform_flag]>>matrix;
        cv::cv2eigen(matrix,matrix_eigen);
        transforms.push_back(matrix_eigen);
        //Read pose flag
        fs[flag]>> pose_flag;
        pose_flags.push_back(pose_flag);
    }
}
Eigen::Vector3f leastSquare3d(const std::vector<Eigen::Matrix4f>& transforms,const std::vector<int>& mask)
{
    //project all transforms to a plane - the transforms should lie on a circular path, project them to a plane to remove vertical noise
    // Plane fitting algorithm found here: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
    //http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
       A <<    0,0,0,
               0,0,0,
               0,0,0;
       b <<    0,0,0;
       for(int i = 0; i<transforms.size(); i++)
       {
           if(mask[i]!=0){
               Eigen::Vector3f p(transforms[i](0,3),transforms[i](1,3),transforms[i](2,3));
               b += Eigen::Vector3f(p(0)*p(2),p(1)*p(2),p(2));
               Eigen::Matrix3f a_tmp;
               a_tmp <<    p(0)*p(0),p(0)*p(1),p(0),
                           p(0)*p(1),p(1)*p(1),p(1),
                           p(0),p(1),1;
               A += a_tmp;
           }
       }
       Eigen::Vector3f x = A.colPivHouseholderQr().solve(b); //plane => x(0)*x + x(1)*y + x(2) = z
   return x;
}
Eigen::Vector3f projectPointToPlane(Eigen::Vector3f p,Eigen::Vector3f x){
    /*plane => x(0)*x + x(1)*y + x(2) = z => x(0)*x + x(1)*y - z  = -x(2)
    / => normal vector:(x(0),x(1),-1), original point(0,0,x(2))
    / https://math.stackexchange.com/questions/444968/project-a-point-in-3d-on-a-given-plane
    */
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    return p3_tmp;
}

Eigen::Matrix4f projectTransformToPlane(Eigen::Matrix4f t,Eigen::Vector3f x){
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);

    float theta = atan(-normal(2)/normal(1));
    float alpha = atan(-(cos(theta)*normal(1) + sin(theta)*normal(2))/normal(0));
    Eigen::Matrix4f transform;

    transform <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),    -origin(0),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),    -origin(1),
                    0,sin(theta),cos(theta),                                    -origin(2),
                    0,0,0,1;

   /* Eigen::Matrix3f roration;
    roration <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),
                    0,sin(theta),cos(theta);
    */
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    Eigen::Matrix4f out(t);
    out(0,3) = p3_tmp(0);
    out(1,3) = p3_tmp(1);
    out(2,3) = p3_tmp(2);
    return transform * out;
}

void projectTransformTranslationsToPlane(Eigen::Vector3f x, std::vector<Eigen::Matrix4f>& transforms, std::vector<int> mask){
    //project the translation component of the transforms onto a plane defined by the parameters stored in 'x'
    float angle = M_PI*2./transforms.size();
   // std::cout << "STEP: " << angle << std::endl;
    Eigen::Vector3f previous;
    Eigen::Matrix4f previous_t;
    float previous_i;
    for(int i = 0; i<transforms.size(); i++)
    {
        Eigen::Matrix4f t = (transforms)[i];
        Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
        p = projectPointToPlane(p,x);

        if(mask[i]!=0){
            previous = p;
            previous_t = t;
            previous_i = i;
        }
        else{
            t = previous_t;
            float o = 0;
            float time = 0;
            Eigen::Vector3f p2;
            Eigen::Matrix4f t2;
            for(int j = i+1; i<transforms.size(); j++){
                if(mask[j]!=0)
                {
                    o = (j-previous_i)*angle;
                    time = ((float)(i-previous_i))/(j-previous_i);
                    t2 = (transforms)[j];
                    p2 << t2(0,3),t2(1,3),t2(2,3);
                    p2 = projectPointToPlane(p2,x);
                    break;
                }
            }
     //   std::cout << "ANGLE: " << o << std::endl;
//      p = previous*(sin((1-time)*o)/sin(o)) + p2*(sin(time*o)/sin(o));

            t = previous_t*(1-time) + t2*time;
            p << t(0,3),t(1,3),t(2,3);
        }
       // std::cout << "Transform: " << p(0) << " " << p(1) << " " << p(2) << " " << std::endl;
        (transforms)[i] <<    t(0,0),t(0,1),t(0,2),p(0),
                            t(1,0),t(1,1),t(1,2),p(1),
                            t(2,0),t(2,1),t(2,2),p(2),
                            t(3,0),t(3,1),t(3,2),t(3,3);
    }
}
