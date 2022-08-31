/**
* @file master.h
* @author Arnaud Meline
* @brief Header file for master.cpp.
* @date October 2018 12.
*/

#pragma once

#include <ethercatcpp/slave.h>
#include <ethercatcpp/ethercat_bus.h>

#include <iostream>
#include <string>
#include <chrono>

#include <cstdlib>
#include <cstring>
#include <cstdio>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  class soem_master_impl; //PImpl to use EtherCAT soem

  /** @brief This class define The EtherCAT Master
  *
  * To create an EtherCAT system, Master is needed. It regroup all datas
  * (network interface used, configurations, devices used, ...) and control the step cycle.
  */
  class Master
  {
      public:

        /**
        * @brief Constructor of Master class
        */
        Master();
        ~Master();

        /**
        * @brief Function used to add a primary network interface to the EtherCAT system (Master)
        *
        * @param [in] interface_primary name of network interface (ex: "eth0").
        */
        void add_Interface_Primary (const std::string& interface_primary);

        /**
        * @brief Function used to add a secondary (redundant) network interface to the EtherCAT system (Master)
        *
        * @param [in] interface_redundant name of network interface (ex: "eth1").
        */
        void add_Interface_Redundant (const std::string& interface_redundant);

        //Function to set conpensate shift time depending of computer speed
        /**
        * @brief Function used to set the DC compensate shift time.
        *
        * The shift time depend on computer speed. A default value is already set
        * to work in most cases (50 ms). If DC synchro don't work and your computer are very low,
        * you can increase this value with a max of 90 ms.
        *
        * @param [in] shift_time shift time value in ms
        */
        void set_DC_Compensate_Shift_Time (int32_t shift_time);

        /**
        * @brief Function used to add a fully define EthercatBus to the EtherCAT system (Master)
        *
        * This function initializes network interface(s) then detect all devices on hardware bus.
        * Then it adds the EthercatBus to the EtherCAT system (master) by matched hardware detected
        * devices and software define devices in EthercatBus. <B> WARNING: the device order in EthercatBus
        * have to be the same at the hardware order. </B>
        *
        * @param [in] ethercatbus fully define EthercatBus (previously define with all devices).
        */
        void add_Bus (EthercatBus& ethercatbus);

        /**
        * @brief Function used to close all interface(s) (ethernet and EtherCAT)
        */
        void end();

        /**
        * @brief Function used to print all devices informations from EtherCAT system (Master) datas
        */
        void print_slave_info();

        /**
        * @brief Function used to launch next cycle.
        *
        * This function send/receive the frame through EtherCAT network.
        */
        bool next_Cycle();

        /**
        * @brief optionnally function used to set the cyclic loop bus period.
        *
        * This function can be used to temporized cyclic loop.
        *
        * Works with "waiting_End_Of_Period" function to wait ending period and must be used just before cyclic loop.
        *
        * @param [in] period_value period of cyclic loop in ms.
        */
        void set_Bus_Period(int period_value);

        /**
        * @brief optionnally function used to wait the end of cyclic loop bus period.
        *
        * This function can be used to waiting the end of cyclic loop period. Must be used in the cyclic loop, just before the end.
        * The period have to be set with "set_Cyclic_Loop_Bus_Period_At" function. If not set, a default period of 5 ms is used.
        *
        */
        void wait_Period();


      //private:

        // copy slave object in slavelist of master
        void add_Slave(int nSlave, Slave* slave_ptr);
        // Update Master vector of slave with info from EthercatBus config
        void update_Init_Masters_From_Slave(int nSlave, Slave* slave_ptr);
        // Update Slave object from Master infos
        void update_Init_Slave_From_Master(int nSlave, Slave* slave_ptr);

        int init_Interface();              //Init ethernet interface
        int init_Interface_Redundant();    //Init ethernet interface for redundant mode
        int init_Network();                //Init network interface(s) and detect all slaves on hardware buss
        int init_Ec_Bus();                 //Init EtherCAT bus (launch bus)
        int init_Bus();                    //Init and config EtherCAT bus (IOmap, slave in OP, ...)

        void init_Canopen_Slaves();        // ask to slave to launch its init canopen configuration sequence.
        void init_IOmap();                 // Configure and create IOmap
        void init_Distributed_Clock();     // init DC on all slave and config dcsync0 if slaved need it
        void activate_Slave_OPstate();     // Set all slave in OP state (EtherCAT ready)
        void update_Slave_config();        // Update Slave IO pointer, FMMU, state.
        void forced_Slave_To_InitState();  // Force slave to Init State
        void forced_Slave_To_PreOp_State();// Force slave to PRE-OP State
        void init_Slaves();                // init steps for slaves
        void end_Slaves();                 // end steps for slaves

        void manage_Ethercat_Error();      // Manage ethercat bus error (print error explicitly)



        soem_master_impl *implmaster_;  //PImpl to use EtherCAT soem

        // Interface definition
        std::string ifname_primary_;    // Network interface definition (eth0, ...)
        char*       ifname_redundant_;  // Redudant network interface definition (eth0, ...) //have to define as char* for soem
        bool        is_redundant_;      // True if redundant network is used.

        uint8_t* iomap_ptr_;         // pointer to IO buffer (IOmap)
        int      expected_wkc_;      // WorkCounter for EtherCAT communication verification

        int max_run_steps_, max_init_steps_, max_end_steps_; // Maximum step that master have to make to update all datas of all slaves
        int max_timewait_steps_; // Maximum time to wait beetween 2 init or end steps (in us)

        std::vector< Slave* > slave_vector_ptr_; // Contain all bus slaves pointer ordered !

        int32_t dc_sync_compensate_shift_time_; // Value to shift first start of DC sync0 time in function of computer speed execution (only in init) in ns

        int cyclic_loop_period_; // time period for a cyclic loop (optionnally) in ms.
        std::chrono::high_resolution_clock::time_point time_point_start_cyclic_loop_; // time point used to temporize cyclic loop.
  };
}
