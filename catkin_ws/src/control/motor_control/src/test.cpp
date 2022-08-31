#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <pid/signal_manager.h>

#include <ethercatcpp/master.h>
#include <ethercatcpp/epos4.h>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <std_msgs/Bool.h>
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


using namespace std;
using namespace std::chrono;
using namespace ethercatcpp;
using namespace pid;

Epos4::control_mode_t control_mode = Epos4::position_CSP;
std::string network_interface_name = "eth0";
double pi = 3.141592653589793;
double rot_in_qc = 1<<18;
double target_value = 0;

static const int period = 25; // ms

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_test");
	ros::NodeHandle n;
	ros::Rate loop_rate(period);

	// Master creation
	Master master_ethercat;

	// Bus creation
	EthercatBus robot;

	// Adding network interface
	master_ethercat.add_Interface_Primary ( network_interface_name );
	cout << "checkpoint 1" << endl;
	// Device definition
	Epos4 epos_1;
	epos_1.set_Id("EPOS4", 0x000000FB, 0x60500000);
	cout << "checkpoint 2" << endl;
	// Linking device to bus in hardware order !!
	robot.add_Device ( epos_1 );
	cout << "checkpoint 3" << endl;
	//add bus to master
	master_ethercat.add_Bus( robot );
	cout << "checkpoint 4" << endl;
	cout << "\n\n\n" << endl;


	while(ros::ok()){

		// Change state of Epos sfm to Lunch power
		if (epos_1.get_Device_State_In_String() == "Switch on disable") {
			epos_1.set_Device_State_Control_Word(Epos4::shutdown);
		}
		if (epos_1.get_Device_State_In_String() == "Ready to switch ON") {
			epos_1.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
		}

		// Set type of control
		epos_1.set_Control_Mode(control_mode);

		if (control_mode == Epos4::position_CSP) {
			if (epos_1.get_Device_State_In_String() == "Operation enable") {
				epos_1.set_Target_Position_In_Rad(target_value);
			}
		}
		if (control_mode == Epos4::profile_position_PPM){
			// unlock axle
			epos_1.halt_Axle(false);
			// Starting new positionning at receive new order (or wait finish before start new with "false state")
			epos_1.change_Starting_New_Pos_Config(true);
			// normal mode (not in endless)
			epos_1.active_Endless_Movement(false);

			//epos_1.active_Absolute_Positionning();
			epos_1.active_Relative_Positionning();

			if (!(epos_1.get_Device_State_In_String() == "Operation enable")) {
				epos_1.activate_Profile_Control(false);
			} else {
				cout << "************************************** \n";
				epos_1.activate_Profile_Control(true);
				epos_1.set_Target_Velocity_In_Rpm(0.000001);
				epos_1.set_Target_Position_In_Rad(target_value);//*131.5);
				cout << "Desired position value = " << std::dec <<target_value << " rad" << "\n";
			}
		}                                                                             

		// epos_1.set_Digital_Output_State(Epos4::dig_out_1, false);
		// epos_1.set_Digital_Output_State(Epos4::dig_out_2, false);
		// epos_1.set_Digital_Output_State(Epos4::dig_out_hs_1, false);

		bool wkc = master_ethercat.next_Cycle();

		if (wkc == true) {

			cout << "State device : " << epos_1.get_Device_State_In_String() << "\n";
			cout << "Control mode = " << epos_1.get_Control_Mode_In_String() << "\n";

			cout << "Actual position : " << std::dec <<epos_1.get_Actual_Position_In_Qc() << " qc" << "\n";
			cout << "Actual position : " << std::dec <<epos_1.get_Actual_Position_In_Rad() << " rad" << "\n";

			cout << "Actual velocity : " << std::dec << epos_1.get_Actual_Velocity_In_Rads() << " rad/s"<<  "\n";
			cout << "Actual Average velocity : " << std::dec << epos_1.get_Actual_Average_Velocity_In_Rads() << " rad/s"<<  "\n";
			cout << "Actual velocity : " << std::dec << epos_1.get_Actual_Velocity_In_Rpm() << " rpm"<<  "\n";
			cout << "Actual Average velocity : " << std::dec << epos_1.get_Actual_Average_Velocity_In_Rpm() << " rpm"<<  "\n";
			cout << "Actual torque : " << std::dec << epos_1.get_Actual_Torque_In_Nm() << " Nm"<< "\n";
			cout << "Actual Average torque : " << std::dec << epos_1.get_Actual_Average_Torque_In_Nm() << " Nm"<< "\n";
			cout << "Actual torque : " << std::dec << epos_1.get_Actual_Torque_In_RT() << " per mile RT"<< "\n";
			cout << "Actual Average torque : " << std::dec << epos_1.get_Actual_Average_Torque_In_RT() << " per mile RT"<< "\n";
			cout << "Actual current : " << std::dec << epos_1.get_Actual_Current_In_A() << " A"<< "\n";
			cout << "Actual Average current : " << std::dec << epos_1.get_Actual_Average_Current_In_A() << " A"<< "\n";

			// Specific for PPM mode
			if (control_mode == Epos4::profile_position_PPM){
				cout << "Target is reached : " << epos_1.check_target_reached() << "\n";
			}

			// cout << "Digital Input 1 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_1) << "\n";
			// cout << "Digital Input 2 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_2) << "\n";
			// cout << "Digital Input 3 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_3) << "\n";
			// cout << "Digital Input 4 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_4) << "\n";
			// cout << "Digital Input Hs 1 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_hs_1) << "\n";
			// cout << "Digital Input Hs 2 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_hs_2) << "\n";
			// cout << "Digital Input Hs 3 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_hs_3) << "\n";
			// cout << "Digital Input Hs 4 = " << epos_1.get_Digital_Input_State(Epos4::dig_in_hs_4) << "\n";

			// cout << "Analog Input 1 = " << epos_1.get_Analog_Input(Epos4::analog_in_1) << " V" << "\n";
			// cout << "Analog Input 2 = " << epos_1.get_Analog_Input(Epos4::analog_in_2) << " V" << "\n";

		} //end of valid workcounter
		ros::spinOnce();
		loop_rate.sleep();
		cout << "\n\n\n" << endl;
	} //end of while

	// close EtherCAT master and socket.
	master_ethercat.end();
	cout << "close master and end of cyclic task " << endl;
	return 0;
}
