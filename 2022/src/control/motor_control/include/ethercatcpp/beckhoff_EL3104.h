/**
* @file beckhoff_EL3104.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EL3104 device.
* @date October 2019 06.
* @example beckhoff_EL3104_example.cpp
*/

#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class describe the EtherCAT driver for a beckhoff EL3104 device
  *
  * This driver permit to communicate with a "beckhoff EL3104" through an EtherCAT bus.
  * The EL3104 EtherCAT Terminal is an interface for the direct connection of 4-channel analog input terminal -10 ... +10 V.
  */
  class EL3104 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EL3104 class
        */
        EL3104();
        ~EL3104() = default;

        //! This enum define open circuit detection pin.
        typedef enum
        {
          channel_1,    //!< Channel 1
          channel_2,    //!< Channel 2
          channel_3,    //!< Channel 3
          channel_4     //!< Channel 4
        }channel_id_t;


        /**
        * @brief Function used to get data value
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return value of specified channel
        */
        int get_Data_Value(channel_id_t channel);

        /**
        * @brief Function used to check underrange detection (Value below measuring range)
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return TRUE if enable, FALSE if disable
        */
        bool check_Underrange(channel_id_t channel);

        /**
        * @brief Function used to check Overrange detection (Measuring range exceeded.)
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return TRUE if enable, FALSE if disable
        */
        bool check_Overrange(channel_id_t channel);

        // /**
        // * @brief Function used to check Limit 1 monitoring
        // *
        // * @param [in] channel desired channel (choose in channel_id_t)
        // * @return 0: not active, 1: value is smaller than limit value 1, 2: value is larger than limit value 1, 3: Value is equal to limit value 1
        // */
        // int check_Limit_1_Monitoring(channel_id_t channel);
        //
        // /**
        // * @brief Function used to check Limit 2 monitoring
        // *
        // * @param [in] channel desired channel (choose in channel_id_t)
        // * @return 0: not active, 1: value is smaller than limit value 1, 2: value is larger than limit value 1, 3: Value is equal to limit value 1
        // */
        // int check_Limit_2_Monitoring(channel_id_t channel);
        // add fct to set limit 1 and 2 value and fct to enable limit 1 and 2 if used


        /**
        * @brief Function used to check Error detection (data invalid)
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return TRUE if NOK, FALSE if OK
        */
        bool check_Error(channel_id_t channel);

        /**
        * @brief Function used to check Sync error detection (no new process data were available)
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return TRUE if NOK, FALSE if OK
        */
        bool check_Sync_Error(channel_id_t channel);

        /**
        * @brief Function used to check current value validity
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return TRUE if invalid, FALSE if valid
        */
        bool check_Value_Validity(channel_id_t channel);

        /**
        * @brief Function used to check current value updating
        *
        * @param [in] channel desired channel (choose in channel_id_t)
        * @return the value is toggled when the data is updated
        */
        bool check_Value_Updating(channel_id_t channel);

        /**
        * @brief Function used to print all datas available
        */
        void print_All_Datas();

      private:

        void unpack_Status_Buffer();




//----------------------------------------------------------------------------//
//                M A I L B O X    D E F I N I T I O N S                      //
//----------------------------------------------------------------------------//

        //Define output mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_out
        {
          int8_t mailbox[128];
        } __attribute__((packed)) mailbox_out_t;
        #pragma pack(pop)

        //Define input mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_in
        {
          int8_t mailbox[128];
        } __attribute__((packed)) mailbox_in_t;
        #pragma pack(pop)

//----------------------------------------------------------------------------//
//                 C Y C L I C    B U F F E R                                 //
//----------------------------------------------------------------------------//

        // typedef int buffer_out_cyclic_command_t[0];

        #pragma pack(push, 1)
        typedef struct buffer_out_cyclic_command
        {
          //No output data
        } __attribute__((packed)) buffer_out_cyclic_command_t;
        #pragma pack(pop)

        #pragma pack(push, 1)
        typedef struct input_data_channel
        {
          uint16_t status_word;
          int16_t data_value;
        } __attribute__((packed)) input_data_channel_t;
        #pragma pack(pop)

        #pragma pack(push, 1)
        typedef struct buffer_in_cyclic_status
        {
          input_data_channel_t channel_1;  //0x6000
          input_data_channel_t channel_2;  //0x6010
          input_data_channel_t channel_3;  //0x6020
          input_data_channel_t channel_4;  //0x6030
        } __attribute__((packed)) buffer_in_cyclic_status_t;
        #pragma pack(pop)

        //Status variable
        input_data_channel_t channel_1_;  //0x6000
        input_data_channel_t channel_2_;  //0x6010
        input_data_channel_t channel_3_;  //0x6020
        input_data_channel_t channel_4_;  //0x6030

  };
}

//Unsued Datas

// bool enable_Limit_1(channel_id_t channel, bool state);
// bool enable_Limit_2(channel_id_t channel, bool state);

// //PDO config for 1 channel, Index 8010 for channel 2, Index 8020 for channel 3, Index 8030 for channel 4
//
//         Index: 8000 Datatype: 0000 Objectcode: 09 Name: AI Settings
//           Sub: 00 Datatype: 0005 Bitlength: 0008 Obj.access: 0007 Name: SubIndex 000
//                   Value :0x18 24
//           Sub: 01 Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable user scale
//                   Value :FALSE
//           Sub: 02 Datatype: 0800 Bitlength: 0003 Obj.access: 033f Name: Presentation
//                   Value :Unknown type
//           Sub: 05 Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Siemens bits
//                   Value :FALSE
//           Sub: 06 Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable filter
//                   Value :FALSE
//           Sub: 07 Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable limit 1
//                   Value :FALSE
//           Sub: 08 Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable limit 2
//                   Value :FALSE
//           Sub: 0a Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable user calibration
//                   Value :FALSE
//           Sub: 0b Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Enable vendor calibration
//                   Value :TRUE
//           Sub: 0e Datatype: 0001 Bitlength: 0001 Obj.access: 033f Name: Swap limit bits
//                   Value :FALSE
//           Sub: 11 Datatype: 0003 Bitlength: 0010 Obj.access: 033f Name: User scale offset
//                   Value :0x0000 0
//           Sub: 12 Datatype: 0004 Bitlength: 0020 Obj.access: 033f Name: User scale gain
//                   Value :0x00010000 65536
//           Sub: 13 Datatype: 0003 Bitlength: 0010 Obj.access: 033f Name: Limit 1
//                   Value :0x0000 0
//           Sub: 14 Datatype: 0003 Bitlength: 0010 Obj.access: 033f Name: Limit 2
//                   Value :0x0000 0
//           Sub: 15 Datatype: 0801 Bitlength: 0010 Obj.access: 033f Name: Filter settings
//                   Value :Unknown type
//           Sub: 17 Datatype: 0003 Bitlength: 0010 Obj.access: 033f Name: User calibration offset
//                   Value :0x0000 0
//           Sub: 18 Datatype: 0003 Bitlength: 0010 Obj.access: 033f Name: User calibration gain
//                   Value :0x4000 16384
//
//
//          Index: 800f Datatype: 0000 Objectcode: 09 Name: AI Vendor data
//           Sub: 00 Datatype: 0005 Bitlength: 0008 Obj.access: 0007 Name: SubIndex 000
//                   Value :0x02 2
//           Sub: 01 Datatype: 0003 Bitlength: 0010 Obj.access: 003f Name: Calibration offset
//                   Value :0xfffffffb -5
//           Sub: 02 Datatype: 0003 Bitlength: 0010 Obj.access: 003f Name: Calibration gain
