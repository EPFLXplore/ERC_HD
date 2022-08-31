/**
* @file epos4.h
* @author Arnaud Meline
* @brief EtherCAT driver for Maxon Epos4 device.
* @date October 2018 12.
* @example epos4_example.cpp
*/

#pragma once
#include <ethercatcpp/ethercat_unit_device.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>


/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {
  /** It permit to communicate with a "Maxon Epos4 control motor" through an ehtercat bus.
  *   @brief This class describe the EtherCAT driver for Maxon Epos4 device
  *
  */

  class Epos4 : public EthercatUnitDevice
  {
      public:

        //! This enum define the "control mode" possibilities
        typedef enum
        {
          no_mode = 0,              //!< No mode seleted
          profile_position_PPM = 1, //!< Profile position mode (PPM)
          profile_velocite_PVM = 3, //!< Profile velocity mode (PVM)
          homing_HMM = 6,           //!< Homing mode (HMM)
          position_CSP = 8,         //!< Cyclic synchronous position mode
          velocity_CSV = 9,         //!< Cyclic synchronous velocity mode
          torque_CST = 10           //!< Cyclic synchronous torque mode
        }control_mode_t;

        //! This enum regroup the "device control state" command
        typedef enum
        {
          shutdown,                //!< Command to active state transition 2, 6, 8
          switch_on,               //!< Command to active state transition 3
          switch_on_and_enable_op, //!< Command to active state transition 3, 4
          disable_voltage,         //!< Command to active state transition 7, 9, 10, 12
          quickstop,               //!< Command to active state transition 11
          disable_op,              //!< Command to active state transition 5
          enable_op,               //!< Command to active state transition 4, 16
        }device_control_state_t;

        //! Digital output name.
        typedef enum
        {
          dig_out_1 = (uint8_t)1,     //!< This output is physicaly mapped on pin 5 on X7 connector
          dig_out_2 = (uint8_t)2,     //!< This output is physicaly mapped on pin 6 on X7 connector
          dig_out_hs_1 = (uint8_t)3   //!< This output is physicaly mapped on pin 5/6 on X6 connector
        }dig_out_id_t;

        //! Digital input name.
        typedef enum
        {
          dig_in_1 = (uint8_t)1,     //!< This input is physicaly mapped on pin 1 on X7 connector
          dig_in_2 = (uint8_t)2,     //!< This input is physicaly mapped on pin 2 on X7 connector
          dig_in_3 = (uint8_t)3,     //!< This input is physicaly mapped on pin 3 on X7 connector
          dig_in_4 = (uint8_t)4,     //!< This input is physicaly mapped on pin 4 on X7 connector
          dig_in_hs_1 = (uint8_t)5,  //!< This input is physicaly mapped on pin 1 on X6 connector and its complement on pin 2
          dig_in_hs_2 = (uint8_t)6,  //!< This input is physicaly mapped on pin 3 on X6 connector and its complement on pin 4
          dig_in_hs_3 = (uint8_t)7,  //!< This input is physicaly mapped on pin 5 on X6 connector and its complement on pin 6
          dig_in_hs_4 = (uint8_t)8   //!< This input is physicaly mapped on pin 7 on X6 connector and its complement on pin 8
        }dig_in_id_t;

        //! Analog input name.
        typedef enum
        {
          analog_in_1,      //!< This input is physicaly mapped on pin 1 for positive signal and pin 2 for negative on X8 connector
          analog_in_2,      //!< This input is physicaly mapped on pin 3 for positive signal and pin 4 for negative on X8 connector
        }analog_in_id_t;


        /**
        * @brief Constructor of Epos4 class
        */
        Epos4();
        ~Epos4() = default;


        /**
        * @brief Function used to set the target position when device
        * is command in position mode.
        *
        * <B>unit: quadcount (qc)</B>.
        * @param [in] target_position is the position desired
        */
        void set_Target_Position_In_Qc (int32_t target_position);

        /**
        * @brief Function used to set the target position when device
        * is command in position mode.
        *
        * <B>unit: radian (rad)</B>.
        * @param [in] target_position is the position desired
        */
        void set_Target_Position_In_Rad (double target_position);

        /**
        * @brief Function used to set the target velocity when device
        * is command in velocity mode.
        *
        * <B>unit: rotation per minute (rpm)</B>.
        * @param [in] target_velocity is the velocity desired
        */
        void set_Target_Velocity_In_Rpm (int32_t target_velocity);

        /**
        * @brief Function used to set the target velocity when device
        * is command in velocity mode.
        *
        * <B>unit: radian per second (rad/s) </B>.
        * @param [in] target_velocity is the velocity desired
        */
        void set_Target_Velocity_In_Rads (double target_velocity);


        /**
        * @brief Function used to set the target torque when device
        * is command in torque mode.
        *
        * <B>unit: per mile rated torque ("per mile" RT)</B>.
        * @param [in] target_torque is the torque desired
        */
        void set_Target_Torque_In_RT (int16_t target_torque);

        /**
        * @brief Function used to set the target torque when device
        * is command in torque mode.
        *
        * <B>unit: Newton meter (Nm)</B>.
        * @param [in] target_torque is the torque desired
        */
        void set_Target_Torque_In_Nm (double target_torque);

        /**
        * @brief Function used to set the position offset.
        *
        * Provides the offset of the target position in Cyclic Synchronous Position Mode
        * <B>unit: quadcount (qc)</B>.
        * @param [in] position_offset is the position offset desired
        */
        void set_Position_Offset_In_Qc (int32_t position_offset);

        /**
        * @brief Function used to set the position offset.
        *
        * Provides the offset of the target position in Cyclic Synchronous Position Mode
        * <B>unit: radian (rad)</B>.
        * @param [in] position_offset is the position offset desired
        */
        void set_Position_Offset_In_Rad (double position_offset);

        /**
        * @brief Function used to set the velocity offset.
        *
        * Provides the offset of the velocity value. In <B>Cyclic Synchronous Position Mode</B>,
        * this object contains the input value for velocity feed forward.
        * In <B>Cyclic Synchronous Velocity Mode</B>, it contains the commanded offset of the drive device.
        * <B>unit: rotation per minute (rpm)</B>.
        * @param [in] velocity_offset is the velocity offset desired
        */
        void set_Velocity_Offset_In_Rpm (int32_t velocity_offset);

        /**
        * @brief Function used to set the velocity offset.
        *
        * Provides the offset of the velocity value. In <B>Cyclic Synchronous Position Mode</B>,
        * this object contains the input value for velocity feed forward.
        * In <B>Cyclic Synchronous Velocity Mode</B>, it contains the commanded offset of the drive device.
        * <B>unit: radian (rad)</B>.
        * @param [in] velocity_offset is the velocity offset desired
        */
        void set_Velocity_Offset_In_Rads (double velocity_offset);

        /**
        * @brief Function used to set the torque offset.
        *
        * Provides the offset for the torque value given in per thousand of Rated Torque.
        * In <B>Cyclic Synchronous Position Mode</B> and Cyclic Synchronous Velocity Mode,
        * the object contains the input value for torque feed forward.
        * In <B>Cyclic Synchronous Torque Mode</B>, it contains the commanded additive
        * torque of the drive, which is added to the Target Torque value.
        * <B>unit: per mile rated torque ("per mile" RT) </B>.
        * @param [in] torque_offset is the torque offset desired
        */
        void set_Torque_Offset_In_RT (int16_t torque_offset);

        /**
        * @brief Function used to set the torque offset.
        *
        * Provides the offset for the torque value given in per thousand of Rated Torque.
        * In <B>Cyclic Synchronous Position Mode</B> and Cyclic Synchronous Velocity Mode,
        * the object contains the input value for torque feed forward.
        * In <B>Cyclic Synchronous Torque Mode</B>, it contains the commanded additive
        * torque of the drive, which is added to the Target Torque value.
        * <B>unit: Newton meter (Nm) </B>.
        * @param [in] torque_offset is the torque offset desired
        */
        void set_Torque_Offset_In_Nm (double torque_offset);

        /**
        * @brief Function used to select the control mode.
        * @param [in] control_mode is the control mode selected
        */
        void set_Control_Mode (control_mode_t control_mode);

        /**
        * @brief Function used to change the state of device.
        * @param [in] state is the device state desire.
        */
        void set_Device_State_Control_Word(device_control_state_t state);



        // Digital/Analog I/O
        /**
        * @brief Function used to set <B>one</B> digital output state.
        * @param [in] id is the output name desired (enumerate in "dig_out_id_t")
        * @param [in] state is the state desired (true or false)
        */
        void set_Digital_Output_State (dig_out_id_t id, bool state);

        /**
        * @brief Function used to get <B>one</B> digital input state.
        * @param [in] id is the input name desired (enumerate in "dig_out_id_t")
        * @return state of input selected
        */
        bool get_Digital_Input_State (dig_in_id_t id);

        /**
        * @brief Function used to get <B>one</B> analog input value.
        * @param [in] id is the input name desired (enumerate in "analog_in_id_t")
        * @return value of input asked in Volt (<B>V</B>)
        */
        double get_Analog_Input(analog_in_id_t id);



        // Status datas
        /***
        * @brief Function used to get actual motor status.
        * @return raw actual status
        */
        uint16_t get_Status_Word();

        /**
        * @brief Function used to get actual motor state.
        * @return explicit actual motor state.
        */
        std::string get_Device_State_In_String();

        /**
        * @brief Function used to get last control mode sended to motor.
        * @return raw control mode
        */
        int8_t get_Control_Mode();

        /**
        * @brief Function used to get last control mode sended to motor.
        * @return explicit actual control mode like in "control_mode_t"
        */
        std::string get_Control_Mode_In_String();

        /**
        * @brief Function used to get actual motor position in quadcount (<B>Qc</B>).
        * @return position value
        */
        int32_t get_Actual_Position_In_Qc();

        /**
        * @brief Function used to get actual motor position in radian (<B>Rad</B>).
        * @return position value
        */
        double get_Actual_Position_In_Rad();

        /**
        * @brief Function used to get actual motor velocity in rotation per minute (<B>rpm</B>).
        * @return velocity value
        */
        int32_t get_Actual_Velocity_In_Rpm();

        /**
        * @brief Function used to get actual averaged motor velocity in rotation per minute (<B>rpm</B>).
        *
        * Represents the velocity actual value filtered by 1st order digital
        * low-pass filter with a cut-off frequency of 50 Hz.
        * @return averaged velocity value
        */
        int32_t get_Actual_Average_Velocity_In_Rpm();

        /**
        * @brief Function used to get actual motor velocity in radian per second (<B>Rad/s</B>).
        * @return velocity value
        */
        double get_Actual_Velocity_In_Rads();

        /**
        * @brief Function used to get actual averaged motor velocity in radian per second (<B>Rad/s</B>).
        *
        * Represents the velocity actual value filtered by 1st order digital
        * low-pass filter with a cut-off frequency of 50 Hz.
        * @return averaged velocity value
        */
        double get_Actual_Average_Velocity_In_Rads();

        /**
        * @brief Function used to get actual motor torque in  "per mile" Rated Torque (<B> "per mile" RT</B>).
        * @return torque value
        */
        int16_t get_Actual_Torque_In_RT();

        /**
        * @brief Function used to get actual averaged motor torque in  "per mile" Rated Torque (<B> "per mile" RT</B>).
        *
        * Represents the torque actual value filtered by 1st order digital
        * low-pass filter with a cut-off frequency of 50 Hz.
        * @return averaged torque value
        */
        int16_t get_Actual_Average_Torque_In_RT();

        /**
        * @brief Function used to get actual motor torque in newton meter (<B>Nm</B>).
        * @return torque value
        */
        double get_Actual_Torque_In_Nm();

        /**
        * @brief Function used to get actual averaged motor torque in newton meter (<B>Nm</B>).
        *
        * Represents the torque actual value filtered by 1st order digital
        * low-pass filter with a cut-off frequency of 50 Hz.
        * @return averaged torque value
        */
        double get_Actual_Average_Torque_In_Nm();

        /**
        * @brief Function used to get actual motor current in amper (<B>A</B>).
        * @return current value
        */
        double get_Actual_Current_In_A();

        /**
        * @brief Function used to get actual averaged motor current in amper (<B>A</B>).
        *
        * Represents the current actual value filtered by 1st order digital
        * low-pass filter with a cut-off frequency of 50 Hz.
        * @return averaged current value
        */
        double get_Actual_Average_Current_In_A();



      //Profile Position Mode (PPM) specific config

      /**
      * @brief Function used to check if target is reached only in profile mode.
      * @return true if target is reach, false in other case
      */
      bool check_target_reached();

      /**
      * @brief Function used to activate control only in profile mode.
      *
      * Activating is valid only when a positive edge is sended.
      * <B>Default</B>: FALSE value: desactivate
      * @param [in] choise : TRUE to activate control, FALSE to desactivate control
      */
      void activate_Profile_Control(bool choise);

      /**
      * @brief Function used to halt motor axle only in profile mode.
      *
      * <B>Default</B>: FALSE: unlocked axle
      * @param [in] choise : TRUE to stop axle, FALSE to execute command.
      */
      void halt_Axle(bool choise);

      /**
      * @brief Function used to select positionning mode only in profile mode position.
      *
      * <B>Default</B>: Relative positionning is activated.
      */
      void active_Absolute_Positionning();

      /**
      * @brief Function used to select positionning mode only in profile mode position.
      *
      * <B>Default</B>: Relative positionning is activated.
      */
      void active_Relative_Positionning();

      /**
      * @brief Function used select action when a new command arrived only in profile mode position.
      *
      * <B>Default</B>: TRUE : interrupt command and restart immediatly
      * @param [in] choise : TRUE to interrupt command and restart immediatly, FALSE to finich actual and start new command after.
      */
      void change_Starting_New_Pos_Config(bool choise);

      /**
      * @brief Function used select action when endless movement command arrived only in profile mode position.
      * @param [in] choise : TRUE to performed an endless movement, FALSE to change nothing (normal operate mode).
      */
      void active_Endless_Movement(bool choise);

      /**
      * @brief Function used to activate power stage.
      *
      * WARNING: This function have to be used in the cyclic loop. Epos need more than 1 cycle to activate its power stage.
      * @return TRUE when power is activate, FALSE when deactivate
      */
      bool activate_Power_Stage();

      /**
      * @brief Function used to deactivate power stage.
      *
      * WARNING: This function have to be used in the cyclic loop. Epos need more than 1 cycle to deactivate its power stage.
      * @return TRUE when power is deactivate, FALSE when activate
      */
      bool deactivate_Power_Stage();

      private:

        bool command_Map_Configuration();
        bool status_Map_Configuration();

        void update_Command_Buffer();
        void unpack_Status_Buffer();

        // Used to change device state in async mode (only on init)
      public:
        void reset_Fault();
      private:
        void read_Rated_Torque();
        void read_Encoder1_Pulses_Nb_Config();
        void read_Digital_Output_Mapping_Configuration();
        void read_Digital_Input_Mapping_Configuration();

//----------------------------------------------------------------------------//
//                B U F F E R S    D E F I N I T I O N S                      //
//----------------------------------------------------------------------------//

        //Define output mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_out
        {
          int8_t mailbox[48];
        } __attribute__((packed)) mailbox_out_t;
        #pragma pack(pop)

        //Define input mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_in
        {
          int8_t mailbox[48];
        } __attribute__((packed)) mailbox_in_t;
        #pragma pack(pop)

//----------------------------------------------------------------------------//
//                 C Y C L I C    B U F F E R                                 //
//----------------------------------------------------------------------------//
        #pragma pack(push, 1)
        typedef struct buffer_out_cyclic_command
        {
          uint16_t control_word;           //name_0x6040_00
          int8_t   operation_modes;        //name_0x6060_00
          int32_t  target_position;        //name_0x607A_00 // in qc (quadcounts = 4x encoder counts / revolution)
          int32_t  target_velocity;        //name_0x60FF_00 // in rpm (revolution per minute)
          int16_t  target_torque;          //name_0x6071_00 // in in "per mile" Motor Rated Torque (RT)
          int32_t  position_offset;        //name_0x60B0_00 // in qc (quadcounts = 4x encoder counts / revolution)
          int32_t  velocity_offset;        //name_0x60B1_00 // in rpm (revolution per minute)
          int16_t  torque_offset;          //name_0x60B2_00 // in in "per mile" RT
          uint32_t digital_output_state;   //name_0x60FE_01
        } __attribute__((packed)) buffer_out_cyclic_command_t;
        #pragma pack(pop)


        #pragma pack(push, 1)
        typedef struct buffer_in_cyclic_status
        {
          uint16_t status_word;                //name_0x6041_00
          int8_t   operation_modes_read;       //name_0x6061_00
          int32_t  actual_position;            //name_0x6064_00 // in qc
          int32_t  actual_velocity;            //name_0x606C_00 // in rpm
          int16_t  actual_torque;              //name_0x6077_00 // in "per mile" RT
          int32_t  actual_current;             //name_0x30D1-2_00 // in mA
          int32_t  actual_average_current;     //name 0x30D1-01 // in mA
          int16_t  actual_average_torque;      //name 0x30D2-01 // in "per mile" RT
          int32_t  actual_average_velocity;    //name 0x30D3-01 // in rpm
          uint32_t digital_input_state;        //name_0x60FD_00
          int16_t  analog_input_1;             //name 0x3160-01 in mV
          int16_t  analog_input_2;             //name 0x3160-02 in mV
        } __attribute__((packed)) buffer_in_cyclic_status_t;
        #pragma pack(pop)



//----------------------------------------------------------------------------//
//                           B U F F E R    D A T A S                         //
//----------------------------------------------------------------------------//

        // Command datas
        uint16_t control_word_;           //name_0x6040_00
        int8_t   control_mode_;           //name_0x6060_00
        int32_t  target_position_;        //name_0x607A_00 // in qc (quadcounts = 4x encoder counts / revolution)
        int32_t  target_velocity_;        //name_0x60FF_00 // in rpm (revolution per minute)
        int16_t  target_torque_;          //name_0x6071_00 // in in "per mile" Motor Rated Torque (RT)
        int32_t  position_offset_;        //name_0x60B0_00 // in qc (quadcounts = 4x encoder counts / revolution)
        int32_t  velocity_offset_;        //name_0x60B1_00 // in rpm (revolution per minute)
        int16_t  torque_offset_;          //name_0x60B2_00 // in in "per mile" RT
        uint32_t digital_output_state_;   //name_0x60FE_01

        // Status datas
        uint16_t status_word_;                //name_0x6041_00
        int8_t   operation_mode_;             //name_0x6061_00
        int32_t  actual_position_;            //name_0x6064_00 // in qc
        int32_t  actual_velocity_;            //name_0x606C_00 // in rpm
        int16_t  actual_torque_;              //name_0x6077_00 // in "per mile" RT
        int32_t  actual_current_;             //name_0x30D1-2_00 // in mA
        int32_t  actual_average_current_;     //name 0x30D1-01 // in mA
        int16_t  actual_average_torque_;      //name 0x30D2-01 // in "per mile" RT
        int32_t  actual_average_velocity_;    //name 0x30D3-01 // in rpm
        uint32_t digital_input_state_;        //name_0x60FD_00
        int16_t  analog_input_1_;             //name 0x3160-01 in mV
        int16_t  analog_input_2_;             //name 0x3160-02 in mV




        // flag of control state device
        static const uint16_t flag_shutdown_set_ = 0x6;   //flag define to set good bit
        static const uint16_t flag_shutdown_unset_ = 0xFF7E; //flag define to unset good bit
        static const uint16_t flag_switch_on_set_ = 0x7;   //flag define to set good bit
        static const uint16_t flag_switch_on_unset_ = 0xFF7F; //flag define to unset good bit
        static const uint16_t flag_disable_voltage_unset_ = 0xFF7D; //flag define to unset good bit
        static const uint16_t flag_quickstop_set_ = 0x2;   //flag define to set good bit
        static const uint16_t flag_quickstop_unset_ = 0xFF7B; //flag define to unset good bit
        static const uint16_t flag_disable_op_set_ = 0x7;   //flag define to set good bit
        static const uint16_t flag_disable_op_unset_ = 0xFF77; //flag define to unset good bit
        static const uint16_t flag_enable_op_set_ = 0xF;   //flag define to set good bit
        static const uint16_t flag_enable_op_unset_ = 0xFF7F; //flag define to unset good bit
        static const uint16_t flag_fault_reset_switch_ = 0x80; //flag define to switch good bit

        // Flag to check State of device in statusword
        static const uint16_t mask_state_device_status_ = 0x006F;
        const std::map<int,std::string> device_state_decode_ = {
            {0x0000,"Not ready to switch on"},
            {0x0040,"Switch on disable"},
            {0x0021,"Ready to switch ON"},
            {0x0023,"Switched ON"},
            {0x0027,"Operation enable"},
            {0x0007,"Quick stop activ"},
            {0x000F,"Fault reaction active"},
            {0x0008,"Fault"} };

        const std::map<int,std::string> control_mode_decode_ = {
            {1,"profile_position_PPM"},
            {3,"profile_velocite_PVM"},
            {6,"homing_HMM"},
            {8,"position_CSP"},
            {9,"velocity_CSV"},
            {10,"torque_CST"} };

        const std::map<uint8_t,std::string> dig_in_string_decode_ = {
            {1,"dig_in_1"},
            {2,"dig_in_2"},
            {3,"dig_in_3"},
            {4,"dig_in_4"},
            {5,"dig_in_hs_1"},
            {6,"dig_in_hs_2"},
            {7,"dig_in_hs_3"},
            {8,"dig_in_hs_4"} };

        const std::map<uint8_t,std::string> dig_out_string_decode_ = {
            {1,"dig_out_1"},
            {2,"dig_out_2"},
            {3,"dig_out_hs_1"} };

        // Digital output datas
        static const int output_number_ = 3;
        std::array<uint8_t, output_number_> digital_output_mapping_ ; // 3 is number of dig output of Epos4
        std::array<uint32_t, output_number_> digital_output_flag_ ; // 3 is number of dig output of Epos4

        // Digital input datas
        static const int input_number_ = 8;
        std::array<uint8_t, input_number_> digital_input_mapping_ ;
        std::array<uint32_t, input_number_> digital_input_flag_ ;

        static constexpr double rads2rpm_ = 60/(2*M_PI); // var to convert rad/s to rpm
        static constexpr double rpm2rads_ = (2*M_PI)/60; // var to convert rpm to rad/s

        uint32_t rated_torque_;   //value of rated torque set by epos4 to decode torque in Nm
        uint32_t encoder1_pulse_nb_config_; // in Qc // value of encoder 1 total pulse number by rotation *4.
        double rad2qc_, qc2rad_;


      };

    }
