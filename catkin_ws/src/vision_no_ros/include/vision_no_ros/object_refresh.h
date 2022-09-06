#ifndef OBJECT_REFRESH_H
#define OBJECT_REFRESH_H
#include <opencv2/opencv.hpp> 
#include <vector>
#include <librealsense2/rs.hpp>
#include <cmath>

#include <vision_no_ros/cntrl_pnl.h>
#include <ros/ros.h>
#include <vision_no_ros/panel_object.h> 

#include <vision_no_ros/quaternion.h>

using namespace std;
using namespace cv;

static float x_pos_average=0;
static float y_pos_average=0;
static float z_pos_average=0;
static float x_rot_average=0;
static float y_rot_average=0;
static float z_rot_average=0;

static float artag_x_pos_average=0;
static float artag_y_pos_average=0;
static float artag_z_pos_average=0;

static float w_quat_average=0;
static float x_quat_average=0;
static float y_quat_average=0;
static float z_quat_average=0;

static int w_positive=0;
static int x_positive=0;
static int y_positive=0;
static int z_positive=0;

static int active_sample=0;

void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                    const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,
                    const rs2_intrinsics& intrinsics, const int& samples);// should maybe add another artgag argument for the objects needing 2 artags at least to localize them...

void refresh_ar_tag_pos(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                        const cntrl_pnl::ArTag& ArTag,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,const int& samples);

float scalar_product_projection_on_axis(const float (&pixel_right) [2],const float (&pixel_left) [2],const float (&axis) [3],const rs2::depth_frame& depth,const rs2_intrinsics& intrinsics);
   
float get_pixel_distance (const Point2f& pixel1,const Point2f& pixel2);

void average_object_params(vision_no_ros::panel_object& object,int samples);

void get_angle_from_polyfit(float& difference);

void draw_object(cv::Mat& image,const vision_no_ros::panel_object& object,const rs2_intrinsics& intrinsics);

float get_distance_to_object(vision_no_ros::panel_object& object,const rs2::depth_frame& depth,const rs2_intrinsics& intrinsics,const float& dist);

void set_sign_of_quat(int& was_positive,const int& num_samples,float& quat,float& quat_avrg );

//#define USE_RS2_PROJECTION

/*
Function refreshes the desired object's params
arguments are:
-object: custom ros message type for a control panel object
-id: ID of the object to be refereshed
-rvecs: rotation vector of the AR tag
-tvecs: translation vector of the AR tag
-ar_1:  reference AR tag struct
-obj:   contol panel object struct of the required object

*/
void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                    const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,
                    const rs2_intrinsics& intrinsics, const int& samples ){ 
// need to create a similar functio that refreshes the active target without seeing any ar tag , first use this one to home onto the active targert then y=use the other function

  for (int i(0);i < ids.size();++i){
    if (ids[i]==ar_1.id){
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=100;  //what  exactly should i do here 
      cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(ar_1,obj);
      float tag_center_x = (corners[i][0].x+corners[i][2].x)/2;// what if you cant see the ar tag anymore which depth to take ??
      float tag_center_y = (corners[i][0].y+corners[i][2].y)/2;
      float dist=depth.get_distance(int(tag_center_x),int(tag_center_y));
      #ifdef USE_RS2_PROJECTION
        float pixel[2]= {tag_center_x,tag_center_y};
        float point[3];
        //center of AR Tag
        rs2_deproject_pixel_to_point(point,&intrinsics,pixel,dist);
        object.x_pos =offset.x_coor+point[0]*1000; //.
        object.y_pos =offset.y_coor-point[1]*1000;//minus because the camera yaxis points down
        object.z_pos=point[2]*1000;
        //projection of horizontal line of ar tag on camera horizontal axis (xaxis positive rightwards)
        float pixel_right [2];
        pixel_right[0]=corners[i][1].x;
        pixel_right[1]=corners[i][1].y;
        float pixel_left [2];
        pixel_left[0]=corners[i][0].x;
        pixel_left[1]=corners[i][0].y;
        float axis [3]= {1,0,0};
        float yaw = scalar_product_projection_on_axis(pixel_right,pixel_left,axis,depth,intrinsics);
        //projection of vertical line of ar tag on camera vertical axis (yaxis positive downwards)
        pixel_right[0]=corners[i][3].x;
        pixel_right[1]=corners[i][3].y;
        pixel_left[0]=corners[i][0].x;
        pixel_left[1]=corners[i][0].y;
        axis[0]=0;
        axis[1]=1;
        float pitch = -scalar_product_projection_on_axis(pixel_right,pixel_left,axis,depth,intrinsics); //add minus sign for pitch to stay coherent with camera corfdinate system
        // if the right edge is higher than the left one then camera needs to rotate on the negative drection to allign but the y axis is positive downwars in the image space!
        float roll =acos((corners[i][1].x-corners[i][0].x)/get_pixel_distance (corners[i][1],corners[i][0]))*180/M_PI;// done with pixels, try doing it in 3d with the same projection algo
        if(corners[i][1].y<corners[i][0].y) roll=-roll;
      #else  //test which method is more accurate
        object.x_pos =offset.x_coor+tvecs[i][0]*1000; //casting and representing the foats with ints cf bens idea...add minus sign 
        object.y_pos =offset.y_coor-tvecs[i][1]*1000;
        object.z_pos =dist*1000;//                 //this is not correct, give depth to the middle pixel use
        //solution with linear fit not ideal 
        float yaw = get_pixel_distance(corners[i][0],corners[i][3])-get_pixel_distance(corners[i][1],corners[i][2]);
        get_angle_from_polyfit(yaw);
        float pitch =get_pixel_distance(corners[i][2],corners[i][3])-get_pixel_distance(corners[i][1],corners[i][0]);
        get_angle_from_polyfit(pitch);
        //float roll=use special projection functions
        //float yaw = acos(get_pixel_distance(corners[i][1],corners[i][0])*0.0014/44)*180/M_PI; //not gonna work use the formula with the projection as I do with z 
        //float pitch = acos(get_pixel_distance(corners[i][1],corners[i][2])*0.0014/44)*180/M_PI;
        float roll =acos((corners[i][1].x-corners[i][0].x)/get_pixel_distance (corners[i][1],corners[i][0]))*180/M_PI;//use the formula and find the angle in the pixel space!This works, just need to adjust the sign
        if(corners[i][1].y<corners[i][0].y) roll=-roll;
      #endif
     
      //get_euler_angle(intrinsics,tag_center_x,tag_center_y,dist);
      //rvecs is a rodrigues angle not a classic euler angle so that sucx do simple geometry to estimate euler angles
      object.x_rot =pitch; //will give the ar tags rotations then the gripper can stay at that angle
      object.y_rot =yaw;//rvecs[i][1]*180/M_PI; //add rotation relative to gripper
      object.z_rot =roll;//rvecs[i][2]*180/M_PI; //add rotation relative to gripper
      object.ar_tag_id =ar_1.id;
      object.x_pos_tag=tvecs[i][0]*1000;
      object.y_pos_tag=tvecs[i][1]*1000; //add minus sign ?
      object.z_pos_tag= dist*1000;
      
      vector<double> quaternion (4);
      convert_rvec_to_quaternion(rvecs[i],quaternion);//quaternions wont be averaged becauae of the +/- transition at 0.7
      object.w_quaternion =quaternion[0];
      object.x_quaternion =quaternion[1];
      object.y_quaternion =quaternion[2];
      object.z_quaternion =quaternion[3];

      average_object_params(object,samples);
      
      break;
    }
    else { //send an error message or a reset arm position command
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=0;
      object.z_pos=depth.get_distance(depth.get_width()/2,depth.get_height()/2); //give the depth of the center pixel
    }
  }
}


void refresh_ar_tag_pos(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                        const cntrl_pnl::ArTag& ArTag,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,const int& samples){
  if (active_sample<samples){
    for (int i(0);i < ids.size();++i){
      if (ids[i]==ArTag.id){
        float tag_center_x = (corners[i][0].x+corners[i][2].x)/2;
        float tag_center_y = (corners[i][0].y+corners[i][2].y)/2;
        float dist=depth.get_distance(int(tag_center_x),int(tag_center_y));
        object.id = ArTag.id;
        object.x_pos=tvecs[i][0]*1000;
        object.y_pos=tvecs[i][1]*1000;
        object.z_pos=dist*1000;
        vector<double> quaternion (4);
        convert_rvec_to_quaternion(rvecs[i],quaternion);
        object.w_quaternion =quaternion[0];
        object.x_quaternion =quaternion[1];
        object.y_quaternion =quaternion[2];
        object.z_quaternion =quaternion[3];
        object.x_rot=0;
        object.y_rot=0;
        object.z_rot=0;
        ++active_sample;
        artag_x_pos_average=artag_x_pos_average+object.x_pos;  
        artag_y_pos_average=artag_y_pos_average+object.y_pos;
        artag_z_pos_average=artag_z_pos_average+object.z_pos;
      }
    }
  }else{
    artag_x_pos_average=artag_x_pos_average/samples;
    artag_y_pos_average=artag_y_pos_average/samples;
    artag_z_pos_average=artag_z_pos_average/samples;
    object.x_pos=artag_x_pos_average;
    object.y_pos=artag_y_pos_average;
    object.z_pos=artag_z_pos_average;
    artag_x_pos_average=0;
    artag_y_pos_average=0;
    artag_z_pos_average=0;
    active_sample=0;
  }
}

float scalar_product_projection_on_axis(const float (&pixel_right) [2],const float (&pixel_left) [2],const float (&axis) [3],const rs2::depth_frame& depth,const rs2_intrinsics& intrinsics){
   
  //right corner of AR tag
  float point_right [3];
  float dist_right=depth.get_distance(pixel_right[0],pixel_right[1]);
  rs2_deproject_pixel_to_point(point_right,&intrinsics,pixel_right,dist_right);
  //float norm_to_right=sqrt(point[0]*point[0]+point[2]*point[2]);
  //float angle_right=asin(norm_to_center/norm_to_right)*180/M_PI;
  //cout<< "sin on the right is : " << norm_to_center/norm_to_right << endl;//sometimes norm to center is bigger thn norm to right or norm to lrft which is probleamtic for the asin, this is becaus ethe triangle i use is not always a rectangular one 
  //left corner of AR tag
  float point_left [3];
  float dist_left=depth.get_distance(pixel_left[0],pixel_left[1]);
  rs2_deproject_pixel_to_point(point_left,&intrinsics,pixel_left,dist_left);
  //float norm_to_left=sqrt(point[0]*point[0]+point[2]*point[2]);
  //float angle_left=asin(norm_to_center/norm_to_left)*180/M_PI;
  //cout<< "sin on the left is : " << norm_to_center/norm_to_left << endl;
  //yaw is right angle - left angle to be positive in anticlockwise rotation
  //float yaw = angle_right-angle_left;
  float vector_left_to_right [3];
  for (int i=0;i<3;++i){
    vector_left_to_right[i]=point_right[i]-point_left[i];
  }
  //scalar product projection on camera x axis for yaw;
  float scalar_product= vector_left_to_right[0]*axis[0]+vector_left_to_right[1]*axis[1]+vector_left_to_right[2]*axis[2];
  float vector_norm = sqrt(vector_left_to_right[0]*vector_left_to_right[0]+vector_left_to_right[1]*vector_left_to_right[1]+vector_left_to_right[2]*vector_left_to_right[2]);
  float angle = acos(scalar_product/vector_norm)*180/M_PI;
  //if left edge is closer than right edge need to rotate in the negative direction for alignment
  if (dist_left<dist_right) angle=-angle;
  return angle;
}

float get_pixel_distance (const Point2f& pixel1,const Point2f& pixel2){
  float x_vector =pixel2.x-pixel1.x;
  float y_vector =pixel2.y-pixel1.y;
  float norm =sqrt(x_vector*x_vector+y_vector*y_vector);
  return norm;
}

void average_object_params(vision_no_ros::panel_object& object,int samples){
  if (active_sample < samples){
    x_pos_average= object.x_pos+x_pos_average;
    y_pos_average= object.y_pos+y_pos_average;
    z_pos_average= object.z_pos+z_pos_average;
    x_rot_average= object.x_rot+x_rot_average;
    y_rot_average= object.y_rot+y_rot_average;
    z_rot_average= object.z_rot+z_rot_average;
    artag_x_pos_average=artag_x_pos_average+object.x_pos_tag;
    artag_y_pos_average=artag_y_pos_average+object.y_pos_tag;
    artag_z_pos_average=artag_z_pos_average+object.z_pos_tag;
    w_quat_average=w_quat_average+fabs(object.w_quaternion);
    x_quat_average=x_quat_average+fabs(object.x_quaternion);
    y_quat_average=y_quat_average+fabs(object.y_quaternion);
    z_quat_average=z_quat_average+fabs(object.z_quaternion);
    set_sign_of_quat(w_positive,samples,object.w_quaternion,w_quat_average);
    set_sign_of_quat(x_positive,samples,object.x_quaternion,x_quat_average);
    set_sign_of_quat(y_positive,samples,object.y_quaternion,y_quat_average);
    set_sign_of_quat(z_positive,samples,object.z_quaternion,z_quat_average);

    ++active_sample;
  }else {
    object.x_pos=x_pos_average/samples;
    object.y_pos=y_pos_average/samples;
    object.z_pos=z_pos_average/samples;
    object.x_rot=x_rot_average/samples;
    object.y_rot=y_rot_average/samples;
    object.z_rot=z_rot_average/samples;
    object.x_pos_tag=artag_x_pos_average/samples;
    object.y_pos_tag=artag_y_pos_average/samples;
    object.z_pos_tag=artag_z_pos_average/samples;
    w_quat_average=w_quat_average/samples;
    x_quat_average=x_quat_average/samples;
    y_quat_average=y_quat_average/samples;
    z_quat_average=z_quat_average/samples;
    set_sign_of_quat(w_positive,samples,object.w_quaternion,w_quat_average);
    set_sign_of_quat(x_positive,samples,object.x_quaternion,x_quat_average);
    set_sign_of_quat(y_positive,samples,object.y_quaternion,y_quat_average);
    set_sign_of_quat(z_positive,samples,object.z_quaternion,z_quat_average);
    x_pos_average=0;
    y_pos_average=0;
    z_pos_average=0;
    x_rot_average=0;
    y_rot_average=0;
    z_rot_average=0;
    artag_x_pos_average=0;
    artag_y_pos_average=0;
    artag_z_pos_average=0;
    w_quat_average=0;
    x_quat_average=0;
    y_quat_average=0;
    z_quat_average=0;
    w_positive=0;
    x_positive=0;
    y_positive=0;
    z_positive=0;
    active_sample=0;
  }
}

void get_angle_from_polyfit(float& difference){ //use differential pixel length to deduce tilt

  
  float angle;
  if(difference>=16) {
    angle=40;
  }else if (difference<=-16){
    angle=-40;
  }else{
    //float angle=2.817*difference-0.7838; //first order is not that bad
    angle =-0.004901*pow(difference,3)-0.00694*pow(difference,2)+3.746*difference+1.405; // looks like a sine so wont be ideal for angles outside of range (-40 to +40 degs)
  }
  difference=angle;
  return;
}


void draw_object(cv::Mat& image,const vision_no_ros::panel_object& object,const rs2_intrinsics& intrinsics){
  float pixel[2];
  float point[3]={object.x_pos,-object.y_pos,object.z_pos}; //added - sign to the object y coordinate beause the axes are inverted (non euclidian repere)
  rs2_project_point_to_pixel(pixel,&intrinsics,point);
  Point center (pixel[0],pixel[1]);
  //cout<< pixel[0] << pixel[1] << endl;
  cv::circle(image,center,20,Scalar(0,255,0),2);
}

void set_sign_of_quat(int& was_positive,const int& num_samples,float& quat,float& quat_avrg ){
  if (active_sample<num_samples){
    if (quat>=0) ++was_positive;
  }else{
    if (was_positive >= num_samples-was_positive) quat=quat_avrg;
    else quat=-quat_avrg;
  }
}




//TRASH



/*

void fix_rotation(const cv::Vec3d& rvec_tag,const cv::Vec3d& tvec_tag,const cv::Vec3d& rvec_obj,const cv::Vec3d& tvec_obj,vision_no_ros::panel_object& object){
cv::Vec3d rvec_cam;
cv::Vec3d tvec_cam;
//tvec
//for (i=0;i<3;++i){
//  tvec_cam[i]=tvec_tag[i]+tvec_tag[i];
//}
//cv::Mat rotation_matrix_obj;
//Rodrigues(rvec_obj,rotation_matrix_obj);
//cv::Mat rotation_matrix_tag;
//Rodrigues(rvec_obj,rotation_matrix_tag);
//cv:Mat rotation_matrix_cam;
composeRT(rvec_obj,tvec_obj,rvec_tag,tvec_tag,rvec_cam,tvec_cam);
object.x_pos=tvec_cam[0];
object.y_pos=tvec_cam[1];
object.z_pos=tvec_cam[2];
cout << "coords are : " << object.x_pos << endl<< object.y_pos << endl << object.z_pos<< endl;

vector<double> quaternion (4);
convert_rvec_to_quaternion(rvec_cam,quaternion);

object.w_quaternion=quaternion[0];
object.x_quaternion=quaternion[1];
object.y_quaternion=quaternion[2];
object.z_quaternion=quaternion[3];
cout << "quaternions are : " << object.w_quaternion << endl<< object.x_quaternion << endl<< object.y_quaternion << endl<< object.z_quaternion << endl;

}





float get_distance_to_object(vision_no_ros::panel_object& object,const rs2::depth_frame& depth,const rs2_intrinsics& intrinsics,const float& dist ){ //was thinking about doinit with pythagoras but finally wont work
  float distance;
  float pixel[2];
  float point[3]={object.x_pos,-object.y_pos,dist};
  cout<< point[0]<<" " << point[1]<< " " << point[2] <<endl;
  rs2_project_point_to_pixel(pixel,&intrinsics,point);
  Point center (pixel[0],pixel[1]);
  cout<< pixel[0] <<" "<< pixel[1] << endl;
  distance=depth.get_distance(pixel[0],pixel[1]);
  cout << distance <<endl;
  return distance;
}
*/
#endif