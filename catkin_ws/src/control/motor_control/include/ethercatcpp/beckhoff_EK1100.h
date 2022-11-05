/**
* @file beckhoff_EK1100.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EK1100 device.
* @date October 2018 12.
*/

#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class is used to add a Beckhoff EK1100 device on EtherCAT buss
  *
  * This device haven't any I/O, it just used in "head" for beckhoff device and used like an EtherCAT switch.
  */
  class EK1100 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EK1100 class
        */
        EK1100();
        ~EK1100() = default;
  };
}
