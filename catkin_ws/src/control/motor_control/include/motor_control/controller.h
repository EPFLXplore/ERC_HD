#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/cache.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <pid/signal_manager.h>
#include <math.h>

#include <ethercatcpp/epos4.h>
#include <ethercatcpp/master.h>
#include <xcontrol_v2/network_master.h>
#include <xcontrol_v2/one_axis_slot.h>
#include <xcontrol_v2/three_axis_slot.h>

class Controller {
public:
    Controller(bool * has_motor);
    void accountForJoint56Dependency();
    void manualCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg):
    void stateCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
    bool command_too_old();
    double security_angle(double vel, size_t it);
    void enforce_limits();
    void update_targets();
    void stop();
    void set_goals();
    void ros_loop();

private:
    double current_pos_[MOTOR_COUNT];
    double target_pos_[MOTOR_COUNT];
    double target_vel_[MOTOR_COUNT];
    bool active_[MOTOR_COUNT];
    double max_current_[MOTOR_COUNT];
    float step_size_[MOTOR_COUNT];
    Epos4::control_mode_t control_mode_;


    bool taking_commands_;
    auto last_command_time_;

    const Epos4::control_mode_t direct_control_mode_;
    const Epos4::control_mode_t automatic_control_mode_;
    const double command_expiration_;
    const float reference_step_size_[MOTOR_COUNT];
    const float max_qc_[MOTOR_COUNT];
    const float min_qc_[MOTOR_COUNT];
    const float max_angle_[MOTOR_COUNT];
    const float min_angle_[MOTOR_COUNT];
    const double max_velocity_[MOTOR_COUNT] = {5, 1, 30, 5};
    const double reduction_[MOTOR_COUNT];
    const double security_angle_coef_[MOTOR_COUNT];

    vector<xcontrol::Epos4Extended*> chain_;
};



#endif