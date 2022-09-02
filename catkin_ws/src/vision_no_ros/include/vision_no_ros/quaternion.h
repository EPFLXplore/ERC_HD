//quaternion_conversion
#ifndef QUATERNION_H
#define QUATERNION_H
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

using namespace cv;
using namespace std;

void convert_rvec_to_quaternion(const cv::Vec3d& rvec, vector<double>& quat);    //double& quat [] is an array of references I believe, not a reference of an array
void getQuaternion(const cv::Mat& R, vector<double>& Q);


void convert_rvec_to_quaternion(const cv::Vec3d& rvec,vector<double>& quat){
    cv::Mat rotation_matrix;
    Rodrigues(rvec,rotation_matrix);
    std::cout << "roatation matrix is : "<< rotation_matrix <<std::endl;
    getQuaternion(rotation_matrix,quat);
    for (size_t i=0 ; i<quat.size();++i){
        cout<<"quaterion coordinates are : " << quat [i] << endl;
    }
}


void getQuaternion(const cv::Mat& R, vector<double>& Q){
     
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;
       
        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

#endif