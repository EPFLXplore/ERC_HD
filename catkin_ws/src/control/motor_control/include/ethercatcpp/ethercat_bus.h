/**
* @file ethercat_bus.h
* @author Arnaud Meline
* @brief Header file for ethercat_bus.cpp.
* @date October 2018 12.
*/

#pragma once

#include <ethercatcpp/ethercat_device.h>



/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class define an EtherCAT bus
  *
  * An EthercatBus is a pool of EthercatDevice who represent a hardware device configuration.
  *
  */
  class EthercatBus
  {
      public:

        /**
        * @brief Constructor of EthercatBus class
        */
        EthercatBus();
        ~EthercatBus() = default;

        /**
        * @brief Function used to add a device (EthercatDevice : unit or agregate device) in the bus.
        *
        * @param [in] device EthercatDevice (unit or agregate) who want to add to the bus.
        */
        void add_Device ( EthercatDevice& device);

        /***
        * @brief Function used to get all "device pointer" who compose the bus.
        *
        * @return a vector of EthercatDevice pointer. This vector regroup all device pointer that are compose the EthercatBus (the vector contain address of all slaves present in the bus).
        */
        std::vector< EthercatDevice* > get_Bus_Device_Vector_Ptr() const;


      private:

        //! This vector contain all EthercatDevice pointer that are composed the EthercatBus. The device order have to be the same of hardware device implantation.
        std::vector< EthercatDevice* > device_bus_ptr_;

  };
}
