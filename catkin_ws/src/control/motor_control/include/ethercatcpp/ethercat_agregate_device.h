/**
* @file ethercat_agregate_device.h
* @author Arnaud Meline
* @brief Header file for ethercat_agregate_device.cpp.
* @date October 2018 12.
*/

#pragma once
#include <ethercatcpp/ethercat_device.h>
#include <ethercatcpp/ethercat_bus.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class define an EtherCAT agregate device
  *
  * An agregate device is composed by EthercatUnitDevice, so it don't have any dedicated Slave
  * (only an EthercatUnitDevice have a dedicated Slave). And, it composed an EthercatBus,
  *  so an agregate device is a bus of EtherCAT device.
  *
  */
  class EthercatAgregateDevice : public EthercatDevice, public EthercatBus
  {
      public:

        /**
        * @brief Constructor of EthercatAgregateDevice class
        */
        EthercatAgregateDevice();
        ~EthercatAgregateDevice() = default;



        /***
        * @brief Function used to get the "device pointer" vector who compose the agregate device
        *
        * @return a vector of EthercatDevice pointer. This vector regroup all device pointer that are compose the EthercatAgregateDevice (the vector contain address of all slaves present in this device).
        */
        std::vector< EthercatDevice* > get_Device_Vector_Ptr() ;

        /***
        * @brief Function used to get the device slave address
        *
        * @return dedicated slave address. Here, a nullptr address is always returned because an agregate device haven't dedicated slave.
        */
        Slave* get_Slave_Address();

  };
}
