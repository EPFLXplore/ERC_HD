#ifndef PLAQUE_DETECTION_H
#define PLAQUE_DETECTION_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs.hpp>

#include <vector>
#include <iostream>

using namespace std;
using namespace cv;
//read image
//transform to grayscale
//transform to binary (tresholding)
//use contour detection to detect the contours of the wghite parts in the image (aka the objects) black pixels are cconsidered background
void find_plaque(const Mat& image,const rs2::depth_frame& depth,const rs2_intrinsics& intrinsics){
    
    Mat image_grey;
    cvtColor(image,image_grey,COLOR_BGR2GRAY);
    
    Mat blurred_image;
    GaussianBlur(image_grey,blurred_image,Size(7,7),0);
   
    Mat image_canny;
    Canny(blurred_image,image_canny, 100, 200);

    Mat thresh_image;
    threshold(image_grey,thresh_image,150,255,THRESH_BINARY);
        
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh_image,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE); //RETR EXTERNAL is used to detect only the external contour of an object (ie just the parent)using chain appprox none is slower because it finds and saves all of the points on the cntour not just the end points
    //drawContours(image,contours,-1,Scalar(0,255,0),2);

    //imshow("canny image",image_canny); //canny and contour detection is impossible using rgb images because both the buttons and the panel are white
    

    vector<vector<Point>> approx_contours;
    approx_contours.resize(contours.size());
    int count_rectangles=0;
    cv::Mat output_image=image.clone();

    for (size_t k=0 ; k<contours.size() ; ++k){
        approxPolyDP(Mat(contours[k]),approx_contours[k],25,true); //the bigger the number the worse the approx which is better for me   // i need a bad approximation for this to work, blurring the edgeds might work I need a low pass filtered image to have the rectangluar shape without any irregu;arities
        if (approx_contours[k].size()==4){
            //for (size_t i=0 ; i<approx_contours.size() ; ++i){

               // circle(output_image,approx_contours[k][i],50,Scalar(0,0,255),1);
            //}
            circle(output_image,approx_contours[k][0],50,Scalar(0,0,255),1);
            circle(output_image,approx_contours[k][2],50,Scalar(255,0,255),1);

            //cout<< "contour number : " << k << "has four sides" <<endl;
            //Point pt1(contours[k][0].x,contours[k][0].y); //contours[k][0];
            //Point pt2(contours[k][2].x,contours[k][2].y); //contours[k][2];
            rectangle(output_image,approx_contours[k][0],approx_contours[k][2],Scalar(0,255,0),5);
            ++count_rectangles;

            ////estimate pose////
            float point_middle_diagonal [3]; //3d point corresponding to the center o the visible rectangle
            float pixel_middle_diagonal [2]= {float((approx_contours[k][0].x+approx_contours[k][2].x)/2),float((approx_contours[k][0].y+approx_contours[k][2].y)/2)};
            float dist_middle_diagonal=depth.get_distance(pixel_middle_diagonal[0],pixel_middle_diagonal[1]);
            rs2_deproject_pixel_to_point(point_middle_diagonal,&intrinsics,pixel_middle_diagonal,dist_middle_diagonal);
            cout << "center of plaque is at : " <<" x = " << point_middle_diagonal[0]<< "        " <<"y = " <<point_middle_diagonal[1] << "      " << "z = "<< point_middle_diagonal[2] <<endl;
        }
    }
    
   
    drawContours(output_image,approx_contours,-1,Scalar(255,0,0),2);
    //cout << contours.size() << endl;
    imshow("detected contours",output_image);

//    imshow("blurred iage",blurred_image);
    
    waitKey(1);
   // destroyAllWindows();

}

#endif
