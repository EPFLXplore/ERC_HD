#include "ros/ros.h"

#include <string>
#include <iostream>
#include <sys/mman.h>
#include <pid/signal_manager.h>
#include <math.h>

#include <ethercatcpp/epos4.h>
#include <ethercatcpp/master.h>
#include <xcontrol_v2/network_master.h>
#include <xcontrol_v2/one_axis_slot.h>
#include <xcontrol_v2/three_axis_slot.h>

#define PRINT_STATE true

using namespace std;
using namespace ethercatcpp;
using namespace pid;

Epos4::control_mode_t control_mode(Epos4::position_CSP);


int main(int argc, char **argv) {

    std::string network_interface_name("eth0");
    ros::init(argc, argv, "hd_controller_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(25);

    xcontrol::ThreeAxisSlot epos_1(true, 0x000000fb, 0x69500000), epos_2(true, 0x000000fb, 0x69500000), epos_3(true, 0x000000fb, 0x69500000);

    vector<xcontrol::Epos4Extended*> chain = {&epos_1, &epos_2, &epos_3};

    bool is_scanning = false; // true;

    xcontrol::NetworkMaster ethercat_master(chain, network_interface_name);

	std::vector<xcontrol::Epos4Extended*> temp;
	for (size_t i = 0; i < chain.size(); i++) {
        temp.push_back(chain[i]);
    }

    cout << "BBBBBBBBBBBBBBBBBBb" << endl;
    ethercat_master.init_network();

    while (ros::ok()){
        ethercat_master.switch_motors_to_enable_op();
        ethercat_master.next_Cycle();
        chain[0]->set_Control_Mode(control_mode);
        chain[1]->set_Control_Mode(control_mode);
        chain[2]->set_Control_Mode(control_mode);
        //chain[1]->set_Target_Position_In_Qc(0);


        ros::spinOnce();
        loop_rate.sleep();
        if (PRINT_STATE) {
            cout << "\n" << endl;
        }
    }
    //definitive_stop(chain);
    cout << "End program" << endl;
    return 0;
}
