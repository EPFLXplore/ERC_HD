/**
* @file fake_device.h
* @author Arnaud Meline
* @brief Header file for fake_device.cpp.
* @date October 2018 12.
*/

#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {
  /** @brief This class define a "fake" EtherCAT device
  *
  * This device haven't any Input/Output. It juste "take a place"
  * on the EtherCAT bus with its manufacturer and model ID.
  * For example a device who is used like a network switch (eg. EK1110).
  */
  class FakeDevice : public EthercatUnitDevice
  {
      public:
        /**
        * @brief Constructor of FakeDevice class
        *
        * @param [in] manufacturer is the device manufacturer id
        * @param [in] model_id is the device model id.
        */
        FakeDevice(uint32_t manufacturer, uint32_t model_id);
        ~FakeDevice();
  };
}
