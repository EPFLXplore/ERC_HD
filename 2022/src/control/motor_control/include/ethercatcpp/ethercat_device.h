/**
* @file ethercat_device.h
* @author Arnaud Meline
* @brief Header file for ethercat_device.cpp.
* @date October 2018 12.
*/

#pragma once
#include <iostream>
#include <string>
#include <vector>

#include <ethercatcpp/slave.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This abstract class define an EtherCAT device.
  *
  * An EtherCAT device can be composed by one (EthercatUnitDevice) or many (EthercatAgregateDevice) device.
  */
  class EthercatDevice
  {
      public:

      EthercatDevice() =default;
      virtual ~EthercatDevice() =default;

      virtual Slave* get_Slave_Address() = 0 ;

      virtual std::vector< EthercatDevice* > get_Device_Vector_Ptr() = 0;

  };

}
