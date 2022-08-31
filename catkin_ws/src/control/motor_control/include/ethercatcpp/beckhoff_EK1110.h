/**
* @file beckhoff_EK1110.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EK1110 device.
* @date October 2018 12.
*/

#pragma once
#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class is used to add a Beckhoff EK1110 device on EtherCAT buss
  *
  * This device haven't any I/O, it just used at the end of a beckhoff device "bus" and used like an EtherCAT switch.
  */
  class EK1110 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EK1110 class
        */
        EK1110();
        ~EK1110() = default;
  };
}
