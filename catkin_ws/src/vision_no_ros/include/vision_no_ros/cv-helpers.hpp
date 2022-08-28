// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <exception>

#include <iostream>

// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    
    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8) //this is the format used by the intel d415
    {
        //std::cout<<"pblm is after this 2"<< std::endl;
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP); //this is the line causing the seg fault when i use cv bridge
        //std::cout<<"pblm is after this 3"<< std::endl;
        cv::cvtColor(r, r,cv::COLOR_BGR2RGB);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}

rs2_intrinsics get_field_of_view(const rs2::pipeline& pipe, cv::Mat& cameraMatrix , cv::Mat& distCoeffs){
        // A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
        // For video streams (streams of images), the sensor that produces the data has a lens and thus has properties such
        //  as a focal point, distortion, and principal point.
        // To get these intrinsics parameters, we need to take a stream and first check if it is a video stream
       
        auto stream = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR);   ///get one of the streams by specifiying the typre check the documentation for all the available stream types the second parameter is -1 by default=> get first availabe profile 
        if (auto video_stream = stream.as<rs2::video_stream_profile>()){
            try{
                //If the stream is indeed a video stream, we can now simply call get_intrinsics()
                rs2_intrinsics intrinsics = video_stream.get_intrinsics();

                auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
                auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
                rs2_distortion model = intrinsics.model;



               // std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
               // std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
               // std::cout << "Distortion Model        : " << model << std::endl;
               // std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
               //     intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
            

                cameraMatrix = (cv::Mat1d(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);   // this is the formatting the camera matrices are expected in 
                distCoeffs = (cv::Mat1d(1, 5) <<intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
                return intrinsics;
            }
            catch (const std::exception& e){
                std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
                //what to return here 
            }
        }
}