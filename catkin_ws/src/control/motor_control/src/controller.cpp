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

using namespace std;
using namespace ethercatcpp;
using namespace pid;


Controller::Controller(bool * has_motor) : {
    double current_pos[MOTOR_COUNT] = {0,0};
    double target_pos[MOTOR_COUNT] = {0,0};
    double target_vel[MOTOR_COUNT] = {0,0};
    bool active[MOTOR_COUNT] = {0,0};
    double max_current[MOTOR_COUNT] = {0.155};
    float step_size[MOTOR_COUNT] = {0};
    Epos4::control_mode_t control_mode(Epos4::velocity_CSV);


    bool taking_commands = true;
    auto last_command_time = chrono::steady_clock::now();

    const Epos4::control_mode_t direct_control_mode = Epos4::velocity_CSV;
    const Epos4::control_mode_t automatic_control_mode = Epos4::profile_position_PPM;
    const double command_expiration = 200;
    const int period = 25;
    const float reference_step_size[MOTOR_COUNT] = {60.0*period};
    const float max_qc[MOTOR_COUNT] = {2*M_PI};
    const float min_qc[MOTOR_COUNT] = {-474270};
    const float max_angle[MOTOR_COUNT] = {3, 1.2, 2, 2};
    const float min_angle[MOTOR_COUNT] = {-6, -0.6, -2, -2};
    const double max_velocity[MOTOR_COUNT] = {5, 1, 30, 5};
    const double reduction[MOTOR_COUNT] = {2*231, 480*16, 243, 2*439};
    const double security_angle_coef[MOTOR_COUNT] = {0, 0, 0, 0};
}

void Controller::accountForJoint56Dependency() {
    if (MOTOR_COUNT >= 6) {
        target_vel[5] -= target_vel[4]*JOINT56_DEPENDENCY;
        // TODO: for position_CSP and profile_position_PPM modes
    }
}


void Controller::manualCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    last_command_time = chrono::steady_clock::now();
    float vel;
    bool empty_command = true;
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        vel = msg->data[it];    // between -1 and 1
        empty_command = empty_command && (vel == 0);
        active[it] = (vel != 0);
        target_vel[it] = double(vel*max_velocity[it]*reduction[it]);
        step_size[it] = double(vel*reference_step_size[it]);
        cout << "received velocity   :   " << target_vel[it] << endl;
    }
    
    if (JOINT56_DEPENDENT) {
        accountForJoint56Dependency();
    }

    if (!empty_command) {
        control_mode = direct_control_mode;
    }
}


void Controller::stateCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    last_command_time = chrono::steady_clock::now();
    control_mode = automatic_control_mode;
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        target_pos[it] = double(msg->position[it]*RAD_TO_QC_CONVERSION);
        target_vel[it] = double(msg->velocity[it]*RAD_TO_QC_CONVERSION); // TODO: maybe I need a different conversion constant here
    }
}


bool Controller::command_too_old() {
    auto now = chrono::steady_clock::now();
    return (chrono::duration_cast<chrono::milliseconds>(now-last_command_time).count() > command_expiration);
}


double Controller::security_angle(double vel, size_t it) {
    // TODO
    if (vel < 0) vel = -vel;
    return vel*security_angle_coef[it];
}


void Controller::enforce_limits(){
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    if(target_pos[it] > max_qc[it]) {
                    cout << "Motor " << it << " target_value over limit: " << std::dec <<target_pos[it] << " qc (max: " << max_qc[it] << ")\n";
                    target_pos[it] = max_qc[it];
                    }
                    if(target_pos[it] < min_qc[it]) {
                        cout << "Motor " << it << " target_value under limit: " << std::dec <<target_pos[it] << " qc (min: " << min_qc[it] << ")\n";
                        target_pos[it] = min_qc[it];
                    }
                    break;

                case Epos4::velocity_CSV:
                    if ((current_pos[it] > max_angle[it]-security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos[it] < min_angle[it]+security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                        target_vel[it] = 0;
                    }
                    break;

                case Epos4::profile_position_PPM:
                    // TODO:
                    break;
            }
        }
    }
}


void Controller::update_targets() {
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            chain[it]->set_Control_Mode(control_mode);

            if (chain[it]->get_Device_State_In_String() == "Operation enable") {
                switch (control_mode) {
                    case Epos4::position_CSP:
                        target_pos[it] += active[it] * step_size[it];
                        break;

                    case Epos4::velocity_CSV:
                        break;

                    case Epos4::profile_position_PPM:
                        // TODO:
                        break;
                }
            }
        }
    }
}


void Controller::stop() {
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    target_pos[it] = current_pos[it];
                    break;

                case Epos4::velocity_CSV:
                    target_vel[it] = 0;
                    break;

                case Epos4::profile_position_PPM:
                    // TODO:
                    break;
            }
        }
    }
}


void Controller::set_goals(){
    if (command_too_old()) {
        stop(chain);
    }
    else {
        update_targets(chain);
    }

    enforce_limits(chain);

    for (size_t it=0; it<chain.size(); ++it) {
        chain[it]->set_Control_Mode(control_mode);
        if (chain[it]->get_has_motor()) {
            chain[it]->set_Control_Mode(control_mode);

            if (chain[it]->get_Device_State_In_String() == "Operation enable") {
                switch (control_mode) {
                    case Epos4::position_CSP:
                        chain[it]->set_Target_Position_In_Qc(target_pos[it]);
                        break;

                    case Epos4::velocity_CSV:
                        chain[it]->set_Target_Velocity_In_Rpm(target_vel[it]);
                        break;

                    case Epos4::profile_position_PPM:
                        // TODO:
                        break;
                }
            }
        }
    }
}


int main(int argc, char **argv) {

    std::string network_interface_name("eth0");
    ros::init(argc, argv, "hd_controller_motors");
    ros::NodeHandle n;
    ros::Subscriber man_cmd_sub = n.subscribe<std_msgs::Float32MultiArray>("/arm_control/manual_cmd", 10, manualCommandCallback);
    ros::Subscriber state_cmd_sub = n.subscribe<sensor_msgs::JointState>("/arm_control/joint_cmd", 10, stateCommandCallback);
    ros::Publisher telem_pub = n.advertise<sensor_msgs::JointState>("/arm_control/joint_telemetry", 1000);
    ros::Rate loop_rate(period);
    cout << "ROS node initialized" << endl;

    // Device definition
    // 3-axis: 1st slot next to ETHERNET-IN
    xcontrol::OneAxisSlot epos_1(true, 0x000000fb, 0x60500000);
    xcontrol::OneAxisSlot epos_2(true, 0x000000fb, 0x65510000);
    xcontrol::ThreeAxisSlot epos_4(true, 0x000000fb, 0x69500000), epos_3(true, 0x000000fb, 0x69500000), empty(false, 0x000000fb, 0x69500000);
    xcontrol::ThreeAxisSlot epos_5(false, 0x000000fb, 0x69500000), epos_6(false, 0x000000fb, 0x69500000), epos_7(false, 0x000000fb, 0x69500000);

    //Epos4Extended epos_1(true);
    //epos_1.set_Id("EPOS4", 0x000000fb, 0x60500000);
    //xcontrol::OneAxisSlot epos_2(true);
    //xcontrol::ThreeAxisSlot epos_2(true), epos_3(true), epos_4(true);
    //xcontrol::ThreeAxisSlot epos_5(true), epos_6(true), epos_7(true);
    vector<xcontrol::Epos4Extended*> chain = {&epos_1, &epos_2, &epos_3, &epos_4, &empty}; //, &epos_5, &epos_6, &epos_7};

    bool is_scanning = true;

    xcontrol::NetworkMaster ethercat_master(chain, network_interface_name);
    cout << "BBBBBBBBBBBBBBBBBBb" << endl;
    ethercat_master.init_network();
    //Master ethercat_master;
    //EthercatBus robot;
    //ethercat_master.add_Interface_Primary(network_interface_name);
    //robot.add_Device(epos_1);
    cout << "AAAAAAAAAAAAAAAAA" << endl;
    //ethercat_master.add_Bus(robot);

    cout << "Ethercat network online" << endl;

    while (ros::ok()){
        // check device status
        ethercat_master.switch_motors_to_enable_op();
        //epos_1.switch_to_enable_op();

        if (!is_scanning && taking_commands) {
            set_goals(chain);
        }

        bool wkc = ethercat_master.next_Cycle(); // Function used to launch next cycle of the EtherCat net
        if (wkc) {
            for (size_t it=0; it<chain.size(); ++it) {
                if (chain[it]->get_has_motor()) {
                    if (PRINT_STATE) {
                        cout << "Motor " << it << "\n";
                        cout << "State device : " << chain[it]->get_Device_State_In_String() << "\n";
                        cout << "Control mode = " << chain[it]->get_Control_Mode_In_String() << "\n";
                    }
                    current_pos[it] = chain[it]->get_Actual_Position_In_Rad()/reduction[it];
                    if (is_scanning) {
                        target_pos[it] = current_pos[it];
                    }
                    if (PRINT_STATE) { 
                        cout << "Actual position : " << std::dec << chain[it]->get_Actual_Position_In_Rad()/reduction[it] << " rad" << "\n";
                        cout << "Actual current value = " << chain[it]->get_Actual_Current_In_A() << "A" << "\n";
                        cout << "\n";
                    }
                }
            }
            is_scanning = false;
        } //end of valid workcounter

        sensor_msgs::JointState msg;
        for (size_t it=0; it<chain.size(); ++it) {
            msg.position.push_back(chain[it]->get_Actual_Position_In_Rad()/reduction[it]);
            msg.velocity.push_back(chain[it]->get_Actual_Velocity_In_Rpm()/reduction[it]);
        }
        telem_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        if (PRINT_STATE) {
            cout << "\n" << endl;
        }
    }
    cout << "End program" << endl;
    return 0;
}
