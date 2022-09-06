#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/aruco.hpp>
#include <vector>
#include <ros/ros.h>

#include <iostream>
#include "std_msgs/Int16.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


//includes for my headers
#include <vision_no_ros/cntrl_pnl.h> //included in  object_refresh
#include <vision_no_ros/cv-helpers.hpp>
#include <vision_no_ros/object_refresh.h>
#include <vision_no_ros/plaque_detection.h>
#include <vision_no_ros/serial_commander.h>
#include <vision_no_ros/gripper_extrinsics.h>
//custom messages includes

#include <vision_no_ros/panel_object.h> //even though this file doesnt exist, the .msg one does
#include <vision_no_ros/object_list.h>


using namespace std;
using namespace cv;

static bool show_input_image(0); //for showing the images directly on the jetson, not through a ros topic
static bool show_output_image(1);//need to turn it on to activate the corresponding ros topic
static bool show_depth_image(0);
#define SAMPLES 30
#define TAG_SIZE 0.05f


////////////////////// vectors required for AR tag detection ///////////////////////////////////
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
static cv::Mat cameraMatrix ;
static cv::Mat distCoeffs ;
static vector<int> ids;
static vector<vector<Point2f> > corners;
static std::vector<cv::Vec3d> rvecs, tvecs;

void fsm_callback(const std_msgs::Int16& msg){
  // this should comtain the ros ok whuile loop but with another condition

        set_command(msg.data); 
}


int main(int argc, char **argv) try {   
    
    //////////// ROS node initialisation ////////////////
    ros::init(argc, argv, "detected_elements_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<vision_no_ros::object_list>("detected_elements", 1);
    ros::Subscriber sub = n.subscribe("fsm_state", 1, fsm_callback);
    
    image_transport::ImageTransport it(n);
    image_transport::Publisher input_image_pub = it.advertise("intel_D405/color_image", 1);
    image_transport::Publisher output_image_pub =it.advertise("intel_D405/color_image_detection",1);
    //ros::spin();//this is needed for the calbacks to be actually called its an infinite loop...

    //////////// control panel initialisation ////////////
    cntrl_pnl::ControlPanel my_panel;
    cntrl_pnl::setup_control_panel(my_panel);
    
    ////////// RealSense pipeline initialisation /////////
    //rs2::context ctx;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device("123622270224"); //devices serial numbers , d405:123622270224, d415:135322062945
    //pipe.start(cfg);
    //setup custom streaming configuration 
    
    cfg.enable_stream(RS2_STREAM_DEPTH,1280, 720, RS2_FORMAT_Z16, 5); //this is the best resolutio for the depth stream to be accurate
    cfg.enable_stream(RS2_STREAM_COLOR,1280, 720, RS2_FORMAT_BGR8, 5);
    pipe.start(cfg);
    
    //pipe.start(); // Start streaming with default recommended configuration//maybe should try to optimze the start parameters for bandwidth gains and other stuff when integratingg
    rs2::align align_to_color(RS2_STREAM_COLOR);//expensive keep it outside the loop


    while (ros::ok()){ // need the loop to keep getting new frames replace the condition with something from the commands i get from the fsm maybe I should add an idle state to this{
        ///////////////get new depth and color frames and align them///////////////////////////
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        data = align_to_color.process(data); //for aligning the depth and color frames
        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth =data.get_depth_frame();
       
        if (show_depth_image){  //ADDING DEPTH VISUALISATION OPTION
            
            rs2::colorizer color_map;
            rs2::frame depth_frame=data.get_depth_frame().apply_filter(color_map);
            //cv::Mat depth_image = frame_to_mat(depth);
             // Query frame size (width and height)
            const int w = depth_frame.as<rs2::video_frame>().get_width();
            const int h = depth_frame.as<rs2::video_frame>().get_height();

            // Create OpenCV matrix of size (w,h) from the colorized depth data
            Mat depth_image(Size(w, h), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
            
            
            imshow("colored depth image",depth_image);
        } 

        ///////////////// camera calibration and image conversion from rs2 frame to cv mat /////////////////////////
        rs2_intrinsics intrinsics = get_field_of_view(pipe,cameraMatrix,distCoeffs); //function to get the camera intrinsics and copy them into the right matrices
        
        cv::Mat image = frame_to_mat(color);  //using the cv helpers to convert an rs2 frame to a cv mat
        if(show_input_image){
            imshow("input feed",image);
            waitKey(1);
        
       
        ////////////converion of cv mat to ros image msg and publishig the video feed on the ros network ///////////////
        //Point pt1(0,0); //contours[k][0];
        //Point pt2(400,400); //contours[k][2];
        //rectangle(image,pt1,pt2,Scalar(255,0,0),2);
        sensor_msgs::ImagePtr input_image_msg;
        input_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        input_image_pub.publish(input_image_msg);
        //cv::waitKey(1);
        //ros::spinOnce();  //not sure cuz there's another one
        }
       
       
        //find_plaque(image,depth,intrinsics);
       

       ////////////////find AR tags ///////////////////////////
        if (get_command()!=-1){
            //apply filters on image to enhance edges and reduce noise ///

            //cvtColor(image, image,CV_BGR2GRAY);
            
            cv::aruco::detectMarkers(image,dictionary,corners,ids);

            


            if (ids.size()>0){

                cv::aruco::estimatePoseSingleMarkers(corners, TAG_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs);// dont forget to modify the ar tag size!! //this function might become obsolete this will need to be added inside the ifs because the ar tag size changes

                //uint command=0;// test variable , replaces the topic I should be subscribed to to know which object to manipulate

                static int active_sample=0;
                ///////////////////////////////////////////////// start refreshing the objects ///////////////////////////////////          
                vision_no_ros::object_list objects;//decalre objects list

                //// panel A

                if (get_command()==0 or get_command()==1){
                    //declare object and refresh it (refresh object function returns coordinates of the camera relative to the ar tag after the last frame, need to call the extrinsics function befor publishibg)
                    vision_no_ros::panel_object main_switch;
                    refresh_object(main_switch,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switchMain,depth,corners,intrinsics,SAMPLES);//need to make a function that gets the rvecs and tvecs for the ar tag with id hard coded
                    //push back the object to the list to be published
                    draw_object(image,main_switch,intrinsics);
                    if (active_sample==SAMPLES && main_switch.reliability!=0){ 
                        //add the gripper extrinsics to the computed camera translation
                        offset_to_fingers(main_switch);
                        objects.detected_objects.push_back(main_switch);
                        //cv::aruco::drawDetectedMarkers(image,corners,ids);
                    }
                }
                if (get_command()==0 or get_command()==2){
                    vision_no_ros::panel_object switch_1;
                    refresh_object(switch_1,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch1,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_1,intrinsics);
                    if (active_sample==SAMPLES && switch_1.reliability!=0){ 
                        offset_to_fingers(switch_1);
                        objects.detected_objects.push_back(switch_1);
                    }
                }

                if (get_command()==0 or get_command()==3){
                    vision_no_ros::panel_object switch_2;
                    refresh_object(switch_2,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch2,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_2,intrinsics);
                    if (active_sample==SAMPLES && switch_2.reliability!=0){ 
                        offset_to_fingers(switch_2);
                        objects.detected_objects.push_back(switch_2);
                    }
                }

                if (get_command()==0 or get_command()==4){
                    vision_no_ros::panel_object switch_3;
                    refresh_object(switch_3,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch3,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_3,intrinsics);
                    if (active_sample==SAMPLES && switch_3.reliability!=0){ 
                        offset_to_fingers(switch_3);
                        objects.detected_objects.push_back(switch_3);
                    }
                }

                if (get_command()==0 or get_command()==5){
                    vision_no_ros::panel_object switch_4;
                    refresh_object(switch_4,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch4,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_4,intrinsics);
                    if (active_sample==SAMPLES && switch_4.reliability!=0){
                        offset_to_fingers(switch_4);
                        objects.detected_objects.push_back(switch_4);
                    }
                }

                if (get_command()==0 or get_command()==6){
                    vision_no_ros::panel_object switch_5;
                    refresh_object(switch_5,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch5,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_5,intrinsics);
                    if (active_sample==SAMPLES && switch_5.reliability!=0){ 
                        offset_to_fingers(switch_5);
                        objects.detected_objects.push_back(switch_5);
                    }
                }

                if (get_command()==0 or get_command()==7){
                    vision_no_ros::panel_object switch_6;
                    refresh_object(switch_6,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch6,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_6,intrinsics);
                    if (active_sample==SAMPLES && switch_6.reliability!=0){ 
                        offset_to_fingers(switch_6);
                        objects.detected_objects.push_back(switch_6);
                    }
                }

                if (get_command()==0 or get_command()==8){
                    vision_no_ros::panel_object switch_7;
                    refresh_object(switch_7,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch7,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_7,intrinsics);
                    if (active_sample==SAMPLES && switch_7.reliability!=0){ 
                        offset_to_fingers(switch_7);
                        objects.detected_objects.push_back(switch_7);
                    }
                }

                if (get_command()==0 or get_command()==9){
                    vision_no_ros::panel_object switch_8;
                    refresh_object(switch_8,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch8,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_8,intrinsics);
                    if (active_sample==SAMPLES && switch_8.reliability!=0) {
                        offset_to_fingers(switch_8);
                        objects.detected_objects.push_back(switch_8);
                    }
                }

                if (get_command()==0 or get_command()==10){
                    vision_no_ros::panel_object switch_9;
                    refresh_object(switch_9,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch9,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,switch_9,intrinsics);
                    if (active_sample==SAMPLES && switch_9.reliability!=0){ 
                        offset_to_fingers(switch_9);
                        objects.detected_objects.push_back(switch_9);
                    }
                }

                //// panel B1

                if (get_command()==0 or get_command()==11){
                    vision_no_ros::panel_object button;
                    refresh_object(button,ids,rvecs,tvecs,my_panel.panelB1.artg2,my_panel.panelB1.button,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,button,intrinsics);
                    if (active_sample==SAMPLES && button.reliability!=0) {
                        offset_to_fingers(button);
                        objects.detected_objects.push_back(button);
                    }
                }

                if (get_command()==0 or get_command()==12){
                    vision_no_ros::panel_object outlet;
                    refresh_object(outlet,ids,rvecs,tvecs,my_panel.panelB1.artg3,my_panel.panelB1.outlet,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,outlet,intrinsics);
                    if (active_sample==SAMPLES && outlet.reliability!=0) {
                        offset_to_voltmeter(outlet);
                        objects.detected_objects.push_back(outlet);
                    }
                }

                if (get_command()==0 or get_command()==13){
                    vision_no_ros::panel_object emagLock;
                    refresh_object(emagLock,ids,rvecs,tvecs,my_panel.panelB1.artg3,my_panel.panelB1.emagLock,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,emagLock,intrinsics);
                    if (active_sample==SAMPLES && emagLock.reliability!=0) {
                        offset_to_fingers(emagLock);
                        objects.detected_objects.push_back(emagLock);
                    }
                }

                //// panel B2

                if (get_command()==0 or get_command()==14){
                    vision_no_ros::panel_object ethernet;
                    refresh_object(ethernet,ids,rvecs,tvecs,my_panel.panelB2.artg4,my_panel.panelB2.ethernet,depth,corners,intrinsics,SAMPLES);
                    draw_object(image,ethernet,intrinsics);
                    if (active_sample==SAMPLES && ethernet.reliability!=0) {
                        offset_to_fingers(ethernet);
                        objects.detected_objects.push_back(ethernet);
                    }
                }

                //////////////////// send AR tag positions and corner limits//////////////////
                
                if (get_command()==0 or get_command()==15){
                    vision_no_ros::panel_object artag_panelA;
                    
                    refresh_ar_tag_pos(artag_panelA,ids,rvecs,tvecs,my_panel.panelA.artg1,depth,corners,SAMPLES);
                    if (active_sample==SAMPLES) {
                        offset_to_fingers(artag_panelA);
                        objects.detected_objects.push_back(artag_panelA);
                    }
                }
                /*
                if (get_command()==0 or get_command()==16){
                   
                   cv::Vec3d hard_code_tvec;
                   hard_code_tvec[0]= distance_from_ARtag(my_panel.panelA.artg1,my_panel.panelA.switchMain).x_coor;
                   hard_code_tvec[1]= distance_from_ARtag(my_panel.panelA.artg1,my_panel.panelA.switchMain).y_coor;
                   hard_code_tvec[2]=0;
                   cv::Vec3d hard_code_rvec;
                   hard_code_rvec[0]=0;
                   hard_code_rvec[1]=0;
                   hard_code_rvec[2]=0;
                   vision_no_ros::panel_object main_switch;
                   fix_rotation(rvecs[0],tvecs[0]/1000,hard_code_rvec,hard_code_tvec,main_switch);
                }
                */
                /////////////////////////////////////////////// end object referesh /////////////////////////////////////////////////

                //cout << "active command is : "<< get_command() <<endl;

                if (active_sample < SAMPLES ){
                    ++active_sample;
                }else {
                    //set_command(); //that way you reask every 30 frames
                    active_sample=0;
                    //publish the list               
                    pub.publish(objects);
                    ros::spinOnce(); // thats necessary her ebecaus it will allow us to callback the fsm function and change states after the 30 frame averaging and not in between
                }


                if (show_output_image){
                    cv::Mat output_image=image.clone();
                    cv::aruco::drawDetectedMarkers(output_image,corners,ids);
                    for(int i=0; i<ids.size(); i++){ 
                      cv::aruco::drawAxis(output_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);// removed this for the jetson (uses a version thats not compatible with this)
                    }  //X: red, Y: green, Z: blue.
                    //add draw objects here
                    sensor_msgs::ImagePtr output_image_msg;
                    output_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",output_image).toImageMsg();
                    output_image_pub.publish(output_image_msg);
                    imshow("output image", output_image);
                    waitKey(1);
                }
            }else {
                cout<< "no visible AR tags" <<endl; // no ar tags are visible call the blind functions
            }
        }ros::spinOnce(); //calls all the callbacks waiting to be called at this time so its needed to use any subscriber callback function
        //close everything (but then how do you reopen )???
    }
    return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
// stop piepline streming

//rs2_pipeline_stop(pipeline, &e);
//    check_error(e);
//
//    // Release resources
//    free(buffer);
//    rs2_delete_pipeline_profile(pipeline_profile);
//    rs2_delete_stream_profiles_list(stream_profile_list);
//    rs2_delete_stream_profile(stream_profile);
//    rs2_delete_config(config);
//    rs2_delete_pipeline(pipeline);
//    rs2_delete_device(dev);
//    rs2_delete_device_list(device_list);
//    rs2_delete_context(ctx);
//
//    return EXIT_SUCCESS;
//}

/*
void publisher_setup() {
 
  ros::init(argc, argv, "detected_elements_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<vision_no_ros::vector_msg>("detected_elements", 1);
 // ros::Rate loop_rate(0.5);
}
*/
