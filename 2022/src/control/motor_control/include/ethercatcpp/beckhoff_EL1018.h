/**
* @file beckhoff_EL1018.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EL1018 device.
* @date February 2020.
* @example beckhoff_EL1018_example.cpp
*/

#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class describe the EtherCAT driver for a beckhoff EL1018 device
  *
  * This driver permit to communicate with a "beckhoff EL1018" through an EtherCAT bus.
  * The EL1018 EtherCAT Terminal is an interface to connect 4-channel digital input (fast).
  *
  * WARNING !! Inputs ids channels are not the same that Connectors ids !!!
  *
  */
  class EL1018 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EL1018 class
        */
        EL1018();
        ~EL1018() = default;

        //! This enum define open circuit detection pin.
        typedef enum
        {
          channel_1,    //!< Channel 1
          channel_2,    //!< Channel 2
          channel_3,    //!< Channel 3
          channel_4,    //!< Channel 4
          channel_5,    //!< Channel 5
          channel_6,    //!< Channel 6
          channel_7,    //!< Channel 7
          channel_8     //!< Channel 8
        }channel_id_t;


        /**
        * @brief Function used to get data value
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return state of specified channel
        */
        bool get_Data_Value(channel_id_t channel);

        /**
        * @brief Function used to print all datas available
        */
        void print_All_Datas();

      private:

        void unpack_Status_Buffer();



//----------------------------------------------------------------------------//
//                 C Y C L I C    B U F F E R                                 //
//----------------------------------------------------------------------------//

        #pragma pack(push, 1)
        typedef struct buffer_in_cyclic_status
        {
          uint8_t data;
        } __attribute__((packed)) buffer_in_cyclic_status_t;
        #pragma pack(pop)

        //Status variable
        uint8_t data_;

  };
}
