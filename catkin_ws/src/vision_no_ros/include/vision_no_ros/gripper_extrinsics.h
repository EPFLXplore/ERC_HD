//gripper extrisics.h
#ifndef GRIPPER_EXTRINSICS_H
#define GRIPPER_EXTRINSICS_H

#include <vector>
#include <vision_no_ros/panel_object.h> 

//distances in mm
#define FINGERS_CAMERA_X 0
#define FINGERS_CAMERA_Y 52
#define FINGERS_CAMERA_Z 109 //remove camera thickness

//73/90

#define VOLTMETER_CAMERA_X 10
#define VOLTMETER_CAMERA_Y 10
#define VOLTMETER_CAMERA_Z 10

void offset_to_fingers(vision_no_ros::panel_object& object);
void offset_to_voltmeter(vision_no_ros::panel_object& object);
void fix_quaternions_for_control(vision_no_ros::panel_object& object);


void offset_to_fingers(vision_no_ros::panel_object& object){
    //add translation to fingers
    object.x_pos+=FINGERS_CAMERA_X;
    object.y_pos+=FINGERS_CAMERA_Y;
    object.z_pos-=FINGERS_CAMERA_Z;

    object.x_pos_tag+=FINGERS_CAMERA_X;
    object.y_pos_tag+=FINGERS_CAMERA_Y;
    object.z_pos_tag-=FINGERS_CAMERA_Z;
   
    
    // fix quaternions
    fix_quaternions_for_control(object);


}

void offset_to_voltmeter(vision_no_ros::panel_object& object){
    //add translation to fingers
    object.x_pos+=VOLTMETER_CAMERA_X;
    object.y_pos+=VOLTMETER_CAMERA_Y;
    object.z_pos+=VOLTMETER_CAMERA_Z;
    
    // fix quaternions
   // fix_quaternions_for_control(object);

    
}

void fix_quaternions_for_control(vision_no_ros::panel_object& object){

    float temp_quat=object.x_quaternion;
    object.x_quaternion=object.z_quaternion;
    object.z_quaternion=temp_quat;


}



#endif