/**
* @file beckhoff_EL2008.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EL2008 device.
* @date February 2020.
* @example beckhoff_EL2008_example.cpp
*/

#pragma once
#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class describe the EtherCAT driver for a beckhoff EL2008 device
  *
  * This driver permit to communicate with a "beckhoff EL2008" through an EtherCAT bus.
  * The EL2008 EtherCAT Terminal is an interface to connect 4-channel digital output.
  *
  * WARNING !! Outputs ids channels are not the same that Connectors ids !!!
  *
  */
  class EL2008 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EL2008 class
        */
        EL2008();
        ~EL2008() = default;

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
        * @brief Function used to set output state
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @param [in] state output desired state
        */
        void set_Output_State(channel_id_t channel, bool state);

      private:

          void update_Command_Buffer();


//----------------------------------------------------------------------------//
//                 C Y C L I C    B U F F E R                                 //
//----------------------------------------------------------------------------//

        #pragma pack(push, 1)
        typedef struct buffer_out_cyclic_command
        {
          uint8_t data = 0;
        } __attribute__((packed)) buffer_out_cyclic_command_t;
        #pragma pack(pop)

        //Data variable
        uint8_t data_;

  };
}
