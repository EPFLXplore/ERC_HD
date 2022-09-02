#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <motor_control/simJointState.h>    // for simulation only

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
//using namespace xcontrol;

#define MOTOR_COUNT 8
#define MAX_MOTOR_COUNT 8
#define PRINT_STATE true
#define QC_SPEED_CONVERSION 4000
#define RAD_TO_QC_CONVERSION 10000
#define JOINT56_DEPENDENCY 1    // TODO
#define JOINT56_DEPENDENT true // TODO
#define PERIOD 25  // [ms]
#define INF 1000000000
#define PI 3.141592653589793
#define ROT_IN_QC 0b1000000000000000000



//====================================================================================================
double current_pos_rad[MAX_MOTOR_COUNT] = {0,0};
double current_pos_qc[MAX_MOTOR_COUNT] = {0};
double target_pos[MAX_MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
double target_vel[MAX_MOTOR_COUNT] = {0,0};
double max_current[MAX_MOTOR_COUNT] = {0.155};
Epos4::control_mode_t control_mode(Epos4::velocity_CSV);

//double offset[MAX_MOTOR_COUNT] = {0};   // TODO: makes the max/min angles unusable -> correct that


bool taking_commands = true;
bool resetting = false;
bool stopped = false;
auto last_command_time = chrono::steady_clock::now();

static const Epos4::control_mode_t direct_control_mode = Epos4::velocity_CSV;
static const Epos4::control_mode_t autonomous_control_mode = Epos4::position_CSP;
static const double command_expiration = 200;
static const int period = 25;
static const float max_angle[MAX_MOTOR_COUNT] = {9.77, 2.3, 411, 9.63, 7.26, INF, 0.395, INF};
static const float min_angle[MAX_MOTOR_COUNT] = {-9.6, -1.393, -259, -9.54, -0.79, -INF, -0.14, -INF};
static const double min_qc[MAX_MOTOR_COUNT] = {-(2<<18), -84000, -1250, -((2<<18)/16), -160000, -((2<<13)/2), -10000000000};
static const double max_qc[MAX_MOTOR_COUNT] = {2<<18, 60000, 1000, (2<<18)/16, 160000, (2<<13)/2, 10000000000};
static const double max_velocity[MAX_MOTOR_COUNT] = {3, 1, 700, 5, 6, 12, 1, 0};    // rotations per minute
//static const double reduction[MAX_MOTOR_COUNT] = {2*231, 480*16, 676.0/49.0, 2*439, 2*439, 2*231, 1*16*700, 0};
static const double reduction[MAX_MOTOR_COUNT] = {1000, 200, 0.5, 50, 50*1.5, 50*1.5, 10000, 100};
static const double full_circle[MAX_MOTOR_COUNT] = {2<<17, 2<<17, 2<<12, 2<<17, 2<<18, 2<<12, 1/2*PI};
static const double rotation_dir_for_moveit[MAX_MOTOR_COUNT] = {1, -1, -1, 1, -1, -1, 1};
static const double security_angle_coef[MAX_MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
static const vector<int> order = {1, 2, 8, 3, 4, 5, 6, 7};

int32_t joint2_offset = 0;

//====================================================================================================



/*
adapt target velocity and position for joint 6 given the ones for joint 5 
*/
void accountForJoint56Dependency() {
    if (MOTOR_COUNT >= 6) {
        double vel = target_vel[4]/reduction[4]*reduction[5];
        target_vel[5] -= vel*JOINT56_DEPENDENCY;
        // TODO: for position_CSP and profile_position_PPM modes
    }
}


void manualCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!taking_commands) return;
    last_command_time = chrono::steady_clock::now();
    float vel;
    bool empty_command = true;
    cout << "received velocity   :";
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        vel = msg->data[it];    // between -1 and 1
        empty_command = empty_command && (vel == 0);
        target_vel[it] = double(vel*max_velocity[it]*reduction[it]*2*PI/60);
        cout << setw(9) << target_vel[it];
    }
    cout << endl;
    
    if (JOINT56_DEPENDENT) {
        //accountForJoint56Dependency();
    }

    if (!empty_command) {
        control_mode = direct_control_mode;
    }
}


/*void resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    taking_commands = !msg->data;
    resetting = msg->data;
}*/


/*
sets zero position of the arm at the current position
*/
/*void set_zero_position() {
    for (size_t it = 0; it < MOTOR_COUNT; it++) {
        offset[it] += current_pos_rad[it];
    }
}*/


/*void setZeroCallback(const std_msgs::Bool::ConstPtr& msg) {
    set_zero_position();
}*/


void stateCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    cout << "================= COMMAND FROM HW INTERFACE ======================" << endl;
    last_command_time = chrono::steady_clock::now();
    control_mode = autonomous_control_mode;
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        cout << "position :     " << msg->position[it] << endl;
        target_pos[it] = int32_t(msg->position[it]*full_circle[it]/2/PI*rotation_dir_for_moveit[it]);
        target_vel[it] = double(msg->velocity[it]/2); //RAD_TO_QC_CONVERSION TODO: maybe I need a different conversion constant here
    }
    target_pos[1] += joint2_offset;
}


/*
indicates if last received command is too old and should be ignored
*/
bool command_too_old() {
    auto now = chrono::steady_clock::now();
    return (chrono::duration_cast<chrono::milliseconds>(now-last_command_time).count() > command_expiration);
}


/*
for velocity mode, calculates an angle at which the velocity must be set to zero in order for the motor not to overshoot its limit
*/
double security_angle(double vel, size_t it) {
    // TODO
    if (vel < 0) vel = -vel;
    return 0*vel*security_angle_coef[it];
}


/*
makes sure the motor stays in its predefined limits (in velocity mode a slight exceeding of the limit may occur)
*/
void enforce_limits(vector<xcontrol::Epos4Extended*> chain){
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    // TODO
                    break;

                case Epos4::velocity_CSV:
                    // TODO
                    if (it == 1) {
                        if ((current_pos_qc[it]-joint2_offset > max_qc[it]-security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos_qc[it]-joint2_offset < min_qc[it]+security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                            target_vel[it] = 0;
                        }
                    }
                    else {
                        if ((current_pos_qc[it] > max_qc[it]-security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos_qc[it] < min_qc[it]+security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                            target_vel[it] = 0;
                        }
                    }
                    break;

                case Epos4::profile_position_PPM:
                    if (target_pos[it] > max_angle[it]) {
                        target_pos[it] = max_angle[it];
                    }
                    if (target_pos[it] < min_angle[it]) {
                        target_pos[it] = min_angle[it];
                    }
                    break;
            }
        }
    }
}


/*
Updates the target positions and velocities in order to stop the motion.
immediate stop is not guaranteed, particularly in velocity mode
*/
void stop(vector<xcontrol::Epos4Extended*> chain) {
    for (size_t it=0; it<chain.size(); ++it) {
        switch (control_mode) {
            case Epos4::position_CSP:
                stopped = true;
                break;
            case Epos4::velocity_CSV:
                target_vel[it] = 0;
                break;
            case Epos4::profile_position_PPM:
                stopped = true;
                break;
        }
    }
}


/*double petit(size_t it, double distance) {
    static const double critical_angle[MAX_MOTOR_COUNT] = {0.087, 0.087, 10.0, 0.087, 0.087, 0.087, 0.087};
    distance = abs(distance);
    double speed = 0.5;
    if (distance > critical_angle[it]) return speed;
    double k = speed/critical_angle[it];
    return distance*k;
}*/


/*
updates target positions and velocities in order to lead the motor to its home (zero) position
*/
/*void reset_position(vector<xcontrol::Epos4Extended*> chain) {
    last_command_time = chrono::steady_clock::now();
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    // TODO:
                    break;

                case Epos4::velocity_CSV:
                    if ((current_pos[it] > -security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos[it] < security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                        target_vel[it] = 0;
                        cout << "QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ" << endl;
                    }
                    else {
                        cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        double dir = 1.0;
                        double distance = current_pos[it];
                        if (current_pos[it] > 0) dir = -1.0;
                        target_vel[it] = dir*petit(it, distance)*max_velocity[it]*reduction[it];
                        cout << "IIIIIIIIIIII" << target_vel[it] << "IIIIIIIIIIIIIIIII" << endl;
                    }
                    break;

                case Epos4::profile_position_PPM:
                    // TODO:
                    break; 
            }
        }
    }
    cout << "IIIIIIIIIIII" << target_vel[0] << "IIIIIIIIIIIIIIIII" << endl;
}*/


void account_for_joint2_home_loss(vector<xcontrol::Epos4Extended*> chain) {
    int32_t pos = chain[1]->get_Actual_Position_In_Qc();
    if (pos > 2<<18) {
        joint2_offset = 2<<19;
        if (target_pos[1] < -2<<18) {
            target_pos[1] += 2<<20;
        }
        else if (target_pos[1] < 2<<18) {
            target_pos[1] += 2<<19;
        }
    }
    else if (pos < -2<<18) {
        joint2_offset = -2<<19;
        if (target_pos[1] > 2<<18) {
            target_pos[1] -= 2<<20;
        }
        else if (target_pos[1] > -2<<18) {
            target_pos[1] -= 2<<19;
        }
    }
    else {
        joint2_offset = 0;
        if (target_pos[1] > 2<<18) {
            target_pos[1] -= 2<<19;
        }
        else if (target_pos[1] < -2<<18) {
            target_pos[1] += 2<<19;
        }
    }
}


/*
gives the target positions and velocities to the motors
*/
void set_goals(vector<xcontrol::Epos4Extended*> chain) {
    stopped = false;
    if (command_too_old()) {
        cout << "COMMAND TOO OLD" << endl;
        //stop(chain);  //TODO: uncomment this
    }
    else if (resetting) {
        cout << "RESETTTTTTTTTTTTTTTTTTTTTTTTTTTTT" << endl;
        //reset_position(chain);
    }
    
    account_for_joint2_home_loss(chain);

    enforce_limits(chain);

    for (size_t it=0; it<chain.size(); ++it) {
        chain[it]->set_Control_Mode(control_mode);
        if (chain[it]->get_has_motor()) {
            chain[it]->set_Control_Mode(control_mode);

            if (chain[it]->get_Device_State_In_String() == "Operation enable") {
                switch (control_mode) {
                    case Epos4::position_CSP:
                        if (!stopped) {
                            if (it == 1) {
                                chain[it]->set_Target_Position_In_Qc(target_pos[it]-joint2_offset);
                            }
                            else {
                                chain[it]->set_Target_Position_In_Qc(target_pos[it]);
                            }
                            cout << "set position       " << target_pos[it] << endl;
                        }
                        break;

                    case Epos4::velocity_CSV:
                        chain[it]->set_Target_Velocity_In_Rads(target_vel[it]);
                        cout << "set velocity       " << target_vel[it] << endl;
                        break;

                    case Epos4::profile_position_PPM:
                        if (!stopped) {
                            // unlock axle
                            chain[it]->halt_Axle(false);
                            // Starting new positionning at receive new order (or wait finish before start new with "false state")
                            chain[it]->change_Starting_New_Pos_Config(true);
                            // normal mode (not in endless)
                            chain[it]->active_Endless_Movement(false);
                            //epos_1.active_Absolute_Positionning();
                            chain[it]->active_Relative_Positionning();
                            chain[it]->activate_Profile_Control(true);
                            //chain[it]->set_Target_Velocity_In_Rpm(target_vel[it]);
                            chain[it]->set_Target_Position_In_Qc(target_pos[it]);
                        }
                        break;
                }
            }
            else {
                //chain[it]->reset_Fault();
            }
        }
    }
}


void definitive_stop(vector<xcontrol::Epos4Extended*> chain) {
    taking_commands = false;
    stop(chain);
    set_goals(chain);
}


int main(int argc, char **argv) {

    std::string network_interface_name("eth0");
    ros::init(argc, argv, "hd_controller_motors");
    ros::NodeHandle n;
    ros::Subscriber man_cmd_sub = n.subscribe<std_msgs::Float32MultiArray>("/arm_control/manual_direct_cmd", 10, manualCommandCallback);
    //ros::Subscriber reset_sub = n.subscribe<std_msgs::Bool>("/arm_control/reset_arm_pos", 10, resetCallback);
    //ros::Subscriber set_zero_sub = n.subscribe<std_msgs::Bool>("/arm_control/set_zero_arm_pos", 10, setZeroCallback);
    ros::Subscriber state_cmd_sub = n.subscribe<sensor_msgs::JointState>("/arm_control/joint_cmd", 10, stateCommandCallback);
    ros::Publisher telem_pub = n.advertise<sensor_msgs::JointState>("/arm_control/joint_telemetry", 1000);
    ros::Publisher sim_telem_pub = n.advertise<motor_control::simJointState>("/arm_control/sim_joint_telemetry", 1000);  // for simulation only
    ros::Rate loop_rate(PERIOD);
    cout << "ROS node initialized" << endl;

    // Device definition
    // 3-axis: 1st slot next to ETHERNET-IN
    xcontrol::OneAxisSlot epos_1(true, 0x000000fb, 0x60500000);
    xcontrol::OneAxisSlot epos_2(true, 0x000000fb, 0x65510000);
    xcontrol::ThreeAxisSlot empty(true, 0x000000fb, 0x69500000), epos_3(true, 0x000000fb, 0x69500000), epos_4(true, 0x000000fb, 0x69500000);
    xcontrol::ThreeAxisSlot epos_6(true, 0x000000fb, 0x69500000), epos_7(true, 0x000000fb, 0x69500000), epos_5(true, 0x000000fb, 0x69500000);

    //Epos4Extended epos_1(true);
    //epos_1.set_Id("EPOS4", 0x000000fb, 0x60500000);
    //xcontrol::OneAxisSlot epos_2(true);
    //xcontrol::ThreeAxisSlot epos_2(true), epos_3(true), epos_4(true);
    //xcontrol::ThreeAxisSlot epos_5(true), epos_6(true), epos_7(true);
    vector<xcontrol::Epos4Extended*> chain = {&epos_1, &epos_2, &empty, &epos_3, &epos_4, &epos_6, &epos_7, &epos_5};

    bool is_scanning = false; // true;

    xcontrol::NetworkMaster ethercat_master(chain, network_interface_name);

	std::vector<xcontrol::Epos4Extended*> temp;
	for (size_t i = 0; i < chain.size(); i++) {
        temp.push_back(chain[i]);
    }
	for (size_t i = 0; i < order.size(); i++) {
        chain[order[i]-1] = temp[i];
    }

    cout << "BBBBBBBBBBBBBBBBBBb" << endl;
    ethercat_master.init_network();
    //Master ethercat_master;
    //EthercatBus robot;
    //ethercat_master.add_Interface_Primary(network_interface_name);
    //robot.add_Device(epos_1);
    cout << "AAAAAAAAAAAAAAAAA" << endl;
    //ethercat_master.add_Bus(robot);

    cout << "Ethercat network online" << endl;

    //sleep(1);
    auto t = chrono::steady_clock::now();

    while (ros::ok()){
        /*auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now-t).count() > 5000) {
            control_mode = Epos4::position_CSP;
        }*/
        // check device status
        ethercat_master.switch_motors_to_enable_op();
        //epos_1.switch_to_enable_op();

        if (!is_scanning) { // && taking_commands) {    TODO
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
                    current_pos_qc[it] = chain[it]->get_Actual_Position_In_Qc();
                    current_pos_rad[it] = chain[it]->get_Actual_Position_In_Qc()/full_circle[it]*2*PI;
                    if (is_scanning) {
                        target_pos[it] = current_pos_qc[it];
                    }
                    if (PRINT_STATE) { 
                        cout << "Actual position : " << std::dec << current_pos_qc[it] << " rad" << "\n";
                        cout << "Actual velocity : " << std::dec << chain[it]->get_Actual_Average_Velocity_In_Rads()/reduction[it] << " rad/s" << "\n";
                        cout << "Actual current value = " << chain[it]->get_Actual_Current_In_A() << "A" << "\n";
                        cout << "\n";
                    }
                }
            }
            is_scanning = false;
        } //end of valid workcounter

        sensor_msgs::JointState msg;
        motor_control::simJointState sim_msg;   // for simulation only
        for (size_t it=0; it<chain.size()-1; ++it) {
            msg.position.push_back(chain[it]->get_Actual_Position_In_Qc()/full_circle[it]*2*PI*rotation_dir_for_moveit[it]);
            msg.velocity.push_back(chain[it]->get_Actual_Velocity_In_Rads()/reduction[it]);
            //sim_msg.position[it] = chain[it]->get_Actual_Position_In_Qc()/reduction[it]*2*PI/ROT_IN_QC;
            //sim_msg.velocity[it] = chain[it]->get_Actual_Velocity_In_Rads()/reduction[it];
        }
        msg.position[6] = 0;
        msg.velocity[6] = 0;
        msg.position[1] -= joint2_offset/full_circle[1]*2*PI*rotation_dir_for_moveit[1];
        if (chain.size() < 6) {
            // populate the message with zeros if less than 6 actual motors
            for (size_t it=chain.size(); it < 6; ++it) {
                msg.position.push_back(0);
                msg.velocity.push_back(0);
                sim_msg.position[it] = 0;
                sim_msg.velocity[it] = 0;
            }
        }
        telem_pub.publish(msg);
        sim_telem_pub.publish(sim_msg);
        ros::spinOnce();
        loop_rate.sleep();
        if (PRINT_STATE) {
            cout << "\n" << endl;
        }
    }
    definitive_stop(chain);
    cout << "End program" << endl;
    return 0;
}
