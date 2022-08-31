#include <xcontrol_v2/network_master.h>

#include <iostream>

using namespace std;
using namespace ethercatcpp;

namespace xcontrol {
  class soem_master_impl; //PImpl to use EtherCAT soem

NetworkMaster::NetworkMaster(vector<Epos4Extended*> chain, std::string network_interface_name) : 
	Master(),
	epos_chain_(chain),
	network_interface_name_(network_interface_name) { }

void NetworkMaster::init_network() {
  	// Bus creation
  	EthercatBus robot;

  	// Adding network interface
  	add_Interface_Primary(network_interface_name_);

	cout << "DDDDDDDD" << endl;
    for (size_t i = 0; i < epos_chain_.size(); i++) {
        robot.add_Device(*(epos_chain_[i]));
		cout << i << "  done" << endl;
    }

	/*init_Interface();
	init_Ec_Bus();
	print_slave_info();*/

  	add_Bus(robot);
}

void NetworkMaster::switch_motors_to_enable_op() {
    for (size_t i = 0; i < epos_chain_.size(); i++) {
        epos_chain_[i]->switch_to_enable_op();
    }
}

void NetworkMaster::sort(std::vector<int> order) {
	std::vector<Epos4Extended*> temp;
	for (size_t i = 0; i < epos_chain_.size(); i++) {
        temp.push_back(epos_chain_[i]);
    }
	for (size_t i = 0; i < order.size(); i++) {
        epos_chain_[order[i]-1] = temp[i];
    }
}


}