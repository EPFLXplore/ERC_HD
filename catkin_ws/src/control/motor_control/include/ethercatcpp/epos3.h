/**
* @file epos3.h
* @author Arnaud Meline
* @brief EtherCAT driver for Maxon Epos3 device.
* @date October 2018 12.
* @example epos3_example.cpp
*/
#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

//TODO ADD a checking firmware to the start (how ?)!

//TODO add PVM, PVT et Home mode ?

//TODO Add HOME mode to make codeur position calib ?
//          mode home -1 to -4  calib with "torque pic"
// Homming mode work in asynch so work with all type of sync buffers

// TODO make fct to use touchprobe (config, accessor, ...)
  // - add fct to decode touch probe status
  // - add fct to encode touch probe fonction

// TODO add specific Config fct for PPM, PVM and PVT
  // finish to decode status word ! (specific mode and fct)
  // add fct to decode interpolation_buffer_status
  // Specific profile position mode (PPM) configurations
  //    -PPM : - motion_profile_type 0x6086 : int16 (0 = trapezoidal || 1 = sinusoidal)
  //           - position_windows 0x6067 : uint32 (range 0 -> 2 147 483 647) || disable = 4 294 967 295 (default)
  //           - position_windows time 0x6068 :
  //    -PVM : - velocity windows 0x606D
  //           - velocity windows time 0x606E
  //    -PVT : - Interpolation sub mode selection : 0x60C0
  //           - Interpolation time period : 0x60C2
  //           - Interpolation data config : 0x60C4
  //           - Interpolation data record : 0x20C1


/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** It permit to communicate with a "Maxon Epos3 control motor" through an ehtercat bus.
  *   @brief This class describe the EtherCAT driver for Maxon Epos3 device
  *
  */
  class Epos3 : public EthercatUnitDevice
  {
      public:

        //! This enum define the general control mode to permit to choose the correct communication buffers
        typedef enum
        {
          profile_mode,             //!< Regroup all profile control mode (position/PPM, velocity/PVM and interpolated position/PTM)
          cyclic_synchronous_mode   //!< Regroup all cyclic synchronous control mode (position/CSP, velocity/CSV and torque/CST )
        }general_control_mode_t;

        //! This enum define the "control mode" possibilities
        typedef enum
        {
          profile_position_PPM = 1,       //!< Profile position mode (PPM)
          profile_velocite_PVM = 3,       //!< Profile velocity mode (PVM)
          interpolated_position_PVT = 7,  //!< Interpolated position mode (PVT)
          homing_HMM = 6,                 //!< Homing mode (HMM)
          position_CSP = 8,               //!< Cyclic synchronous position mode
          velocity_CSV = 9,               //!< Cyclic synchronous velocity mode
          torque_CST = 10                 //!< Cyclic synchronous torque mode
        }control_mode_t;

        //! This enum regroup the "device control state" command
        typedef enum
        {
          shutdown,                //!< Command to active state transition 2, 6, 8
          switch_on,               //!< Command to active state transition 3
          switch_on_and_enable_op, //!< Command to active state transition 3, 4
          disable_voltage,         //!< Command to active state transition 7, 9, 10, 12
          quickstop,               //!< Command to active state transition 7, 10, 11
          disable_op,              //!< Command to active state transition 5
          enable_op,               //!< Command to active state transition 4, 16
          //fault_reset can't be used when device is in OP mode
        }device_control_state_t;

        //! Digital output name.
        typedef enum
        {
          dig_out_1 = (uint8_t)1,  //!< This output is physicaly mapped on pin 13 on J6 connector (signal2)
          dig_out_2 = (uint8_t)2,  //!< This output is physicaly mapped on pin 12 on J6 connector (signal2)
          dig_out_3 = (uint8_t)3,  //!< This output is physicaly mapped on pin 11 on J6 connector (signal2)
          dig_out_4 = (uint8_t)4,  //!< This output is physicaly mapped on pin 10 on J6 connector (signal2)
          dig_out_5 = (uint8_t)5   //!< This output is physicaly mapped on pin 12 on J5 connector (signal1) and its complement on pin 11
        }dig_out_id_t;

        //! Digital input name.
        typedef enum
        {
          dig_in_1 = (uint8_t)1,   //!< This input is physicaly mapped on pin 8 on J6 connector (signal2)
          dig_in_2 = (uint8_t)2,   //!< This input is physicaly mapped on pin 7 on J6 connector (signal2)
          dig_in_3 = (uint8_t)3,   //!< This input is physicaly mapped on pin 6 on J6 connector (signal2)
          dig_in_4 = (uint8_t)4,   //!< This input is physicaly mapped on pin 5 on J6 connector (signal2)
          dig_in_5 = (uint8_t)5,   //!< This input is physicaly mapped on pin 4 on J6 connector (signal2)
          dig_in_6 = (uint8_t)6,   //!< This input is physicaly mapped on pin 3 on J6 connector (signal2)
          dig_in_7 = (uint8_t)7,   //!< This input is physicaly mapped on pin 6 on J5 connector (signal1) and its complement on pin 5
          dig_in_8 = (uint8_t)8,   //!< This input is physicaly mapped on pin 8 on J5 connector (signal1) and its complement on pin 7
          dig_in_9 = (uint8_t)9,   //!< This input is physicaly mapped on pin 4 on J5 connector (signal1) and its complement on pin 2
          dig_in_10 = (uint8_t)10  //!< This input is physicaly mapped on pin 2 on J5 connector (signal1) and its complement on pin 1
        }dig_in_id_t;

        //! Analog input name.
        typedef enum
        {
          analog_in_1,        //!< This output is physicaly mapped on pin 8 on J6 connector (signal2)
          analog_in_2,        //!< This output is physicaly mapped on pin 8 on J6 connector (signal2)
        }analog_in_id_t;

        /**
        * Constructor of Epos3 class
        * @brief The constructor have to have an information on witch mode is used (profile or cyclic mode)
        * @param [in] general_control_mode is general mode selected
        */
        Epos3(general_control_mode_t general_control_mode);
        ~Epos3() = default;


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
        * @return true if selected mode is valid, false otherwise
        */
        bool set_Control_Mode (control_mode_t control_mode);

        /**
        * @brief Function used to change the state of device.
        * @param [in] state is the device state desire.
        */
        void set_Device_State_Control_Word(device_control_state_t state);

        /**
        * @brief Function used to set <B>one</B> digital output state.
        * @param [in] id is the output name desired (enumerate in "dig_out_id_t")
        * @param [in] state is the state desired (true or false)
        */
        void set_Digital_Output_State (dig_out_id_t id, bool state);


        // Status end-user fct

        /**
        * @brief Function used to get <B>one</B> digital input state.
        * @param [in] id is the input name desired (enumerate in "dig_out_id_t")
        * @return state of input selected
        */
        bool get_Digital_Input_State (dig_in_id_t id);

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
        * @brief Function used to get the position value of the touch probe 1 at positive edge detection.
        * @return position value"
        */
        int32_t get_Touch_Probe_Position_Pos();

        /**
        * @brief Function used to get the position value of the touch probe 1 at negative edge detection.
        * @return position value"
        */
        int32_t get_Touch_Probe_Position_Neg();

        /**
        * @brief Function used to get the actual following error in quadcount (<B>Qc</B>) .
        * @return following error value"
        */
        int16_t get_Actual_Following_Error_In_Qc();

        /**
        * @brief Function used to get the actual following error in radian (<B>Rad</B>) .
        * @return following error value"
        */
        double get_Actual_Following_Error_In_Rad();




        // Analog Input / Output
        /**
        * @brief Function used to active used of <B>one</B> analog input.
        *
        * <B>WARNING</B>: analog input increase drasticaly loop time.
        * (from 1 ms to 25 ms for only 1 analog input)
        * @param [in] id is the input name desired (enumerate in "analog_in_id_t")
        */
        void   active_Analog_Input(analog_in_id_t id);

        /**
        * @brief Function used to get <B>one</B> analog input value.
        * @param [in] id is the input name desired (enumerate in "analog_in_id_t")
        * @return value of input asked in Volt (<B>V</B>)
        */
        double get_Analog_Input(analog_in_id_t id);

        /**
        * @brief Function used to active analog output.
        */
        void   active_Analog_Output();

        /**
        * @brief Function used to set analog output value.
        * @param [in] value is the output value desired in Volt (<B>V</B>)
        */
        void   set_Analog_Output(double value);





        // End-user configurations functions
        /**
        * @brief Function used to set acceleration used in profile mode.
        * @param [in] value is acceleration desired in rotation per minute per second (<B>rpm/s</B>)
        */
        void set_Profile_Acceleration_In_Rpms(uint32_t value);
        /**
        * @brief Function used to get acceleration used in profile mode.
        * @return value of acceleration in rotation per minute per second (<B>rpm/s</B>)
        */
        uint32_t get_Profile_Acceleration_In_Rpms();

        /**
        * @brief Function used to set acceleration used in profile mode.
        * @param [in] value is acceleration desired in radian per second squared(<B>rad/s/s</B>)
        */
        void set_Profile_Acceleration_In_Radss(double value);

        /**
        * @brief Function used to get acceleration used in profile mode.
        * @return value of acceleration in radian per second squared(<B>rad/s/s</B>)
        */
        double get_Profile_Acceleration_In_Radss();

        /**
        * @brief Function used to set deceleration used in profile mode.
        * @param [in] value is deceleration desired in rotation per minute per second (<B>rpm/s</B>)
        */
        void     set_Profile_Deceleration_In_Rpms(uint32_t value);

        /**
        * @brief Function used to get deceleration used in profile mode.
        * @return value of deceleration in rotation per minute per second (<B>rpm/s</B>)
        */
        uint32_t get_Profile_Deceleration_In_Rpms();

        /**
        * @brief Function used to set deceleration used in profile mode.
        * @param [in] value is deceleration desired in radian per second squared(<B>rad/s/s</B>)
        */
        void     set_Profile_Deceleration_In_Radss(double value);

        /**
        * @brief Function used to get deceleration used in profile mode.
        * @return value of deceleration in radian per second squared(<B>rad/s/s</B>)
        */
        double   get_Profile_Deceleration_In_Radss(); // in rad/s/s

        /**
        * @brief Function used to set velocity used in profile mode.
        * @param [in] value is velocity desired in rotation per minute (<B>rpm</B>)
        */
        void     set_Profile_Velocity_In_Rpm(uint32_t value);

        /**
        * @brief Function used to get velocity used in profile mode.
        * @return value of velocity in rotation per minute (<B>rpm</B>)
        */
        uint32_t get_Profile_Velocity_In_Rpm();

        /**
        * @brief Function used to set velocity used in profile mode.
        * @param [in] value is velocity desired in radian per second (<B>rad/s</B>)
        */
        void     set_Profile_Velocity_In_Rads(double value);

        /**
        * @brief Function used to get velocity used in profile mode.
        * @return value of velocity in radian per second (<B>rad/s</B>)
        */
        double   get_Profile_Velocity_In_Rads();

        // Profile Mode specific config of control_word
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

        // Profile Position Mode (PPM)
        /**
        * @brief Function used select action when a new command arrived only in profile mode position.
        *
        * <B>Default</B>: TRUE : interrupt command and restart immediatly
        * @param [in] choise : TRUE to interrupt command and restart immediatly, FALSE to finich actual and start new command after.
        */
        void change_Starting_New_Pos_Config(bool choise);

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
        * @brief Function used to set the touch probe function.
        * @param [in] touch_probe_funct is the raw value
        */
        void set_Touch_Probe_funct (uint16_t touch_probe_funct);

        /**
        * @brief Function used to get the touch probe status.
        * @return the raw status value
        */
        uint16_t get_Touch_Probe_Status();

        /**
        * @brief Function used to get the interpolated buffer status.
        * @return the raw interpolated buffer value
        */
        uint16_t get_Interpolation_Buffer_Status();

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

        void update_Command_Buffer();
        void unpack_Status_Buffer();
        //Calcul of averaged value like explain in Epos3 firmware doc
        double calculate_Average_value(double actual_value, double past_average_value);

        // Used to change device state in async mode (only on init)
        void reset_Fault();

        void read_Digital_Output_Mapping_Configuration();
        void read_Digital_Input_Mapping_Configuration();
        void read_Rated_Torque();
        void read_Encoder1_Pulses_Nb_Config();
        void read_Profile_Config();

//----------------------------------------------------------------------------//
//                B U F F E R S    D E F I N I T I O N S                      //
//----------------------------------------------------------------------------//

        //Define output mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_out
        {
          int8_t mailbox[1024];
        } __attribute__((packed)) mailbox_out_t;
        #pragma pack(pop)

        //Define input mailbox size
        #pragma pack(push, 1)
        typedef struct mailbox_in
        {
          int8_t mailbox[1024];
        } __attribute__((packed)) mailbox_in_t;
        #pragma pack(pop)

//----------------------------------------------------------------------------//
//                 C Y C L I C    B U F F E R                                 //
//----------------------------------------------------------------------------//
        #pragma pack(push, 1)
        typedef struct buffer_out_cyclic_command
        {
          uint16_t control_word;           //name_0x6040_00
          int32_t  target_position;        //name_0x607A_00 // in qc (quadcounts = 4x encoder counts / revolution)
          int32_t  target_velocity;        //name_0x60FF_00 // in rpm (revolution per minute)
          int16_t  target_torque;          //name_0x6071_00 // in in "per mile" RT
          int32_t  position_offset;        //name_0x60B0_00 // in qc (quadcounts = 4x encoder counts / revolution)
          int32_t  velocity_offset;        //name_0x60B1_00 // in rpm (revolution per minute)
          int16_t  torque_offset;          //name_0x60B2_00 // in in "per mile" RT
          int8_t   operation_modes;        //name_0x6060_00
          uint16_t digital_output_state;   //name_0x2078_01
          uint16_t touch_probe_funct;      //name_0x60B8_00
          //double bug;
        } __attribute__((packed)) buffer_out_cyclic_command_t;
        #pragma pack(pop)

        #pragma pack(push, 1)
        typedef struct buffer_in_cyclic_status
        {
          uint16_t status_word;                //name_0x6041_00
          int32_t  actual_position;            //name_0x6064_00 // in qc
          int32_t  actual_velocity;            //name_0x606C_00 // in rpm
          int16_t  actual_torque;              //name_0x6077_00 // in "per mile" RT
          int8_t   operation_modes_read;       //name_0x6061_00
          uint16_t digital_input_state;        //name_0x2071_01
          uint16_t touch_probe_status;         //name_0x60B9_00
          int32_t  touch_probe_position_pos;   //name_0x60BA_00
          int32_t  touch_probe_position_neg;   //name_0x60BB_00
        } __attribute__((packed)) buffer_in_cyclic_status_t;
        #pragma pack(pop)

//----------------------------------------------------------------------------//
//               P R O F I L E    B U F F E R                                 //
//----------------------------------------------------------------------------//
        #pragma pack(push, 1)
        typedef struct buffer_out_profile_command
        {
          uint16_t control_word;           //name_0x6040_00
          int32_t  target_position;        //name_0x607A_00 // in qc (quadcounts = 4x encoder counts / revolution)
          int32_t  target_velocity;        //name_0x60FF_00 // in rpm (revolution per minute)
          uint32_t profile_acceleration;   //name 0x6083_00 // in rpm/s
          uint32_t profile_deceleration;   //name_0x6084_00 // in rpm/s
          uint32_t profile_velocity;       //name 0x6081_00 // in rpm
          int8_t   operation_modes;        //name_0x6060_00
          uint16_t digital_output_state;   //name_0x2078_01
        } __attribute__((packed)) buffer_out_profile_command_t;
        #pragma pack(pop)

        #pragma pack(push, 1)
        typedef struct buffer_in_profile_status
        {
          uint16_t status_word;                 //name_0x6041_00
          int32_t  actual_position;             //name_0x6064_00 // in qc
          int32_t  actual_velocity;             //name_0x606C_00 // in rpm
          int16_t  actual_current;              //name_0x6078_00 // in mA
          int16_t  actual_following_error;      //name 0x20F4_00 // in qc
          int8_t   operation_modes_read;        //name_0x6061_00
          uint16_t digital_input_state;         //name_0x2071_01
          uint16_t interpolation_buffer_status; //name 0x20C4_00
        } __attribute__((packed)) buffer_in_profile_status_t;
        #pragma pack(pop)


//----------------------------------------------------------------------------//
//             O U T P U T S   B U F F E R    D A T A S                       //
//----------------------------------------------------------------------------//

        // Datas present in profil and cyclic mode
        int8_t   control_mode_;           //name_0x6060_00
        uint16_t control_word_;           //name_0x6040_00
        int32_t  target_position_;        //name_0x607A_00  // in qc (quadcounts = 4x encoder counts / revolution)
        int32_t  target_velocity_;        //name_0x60FF_00  // in rpm (revolution per minute)
        uint16_t digital_output_state_;   //name_0x2078_01

        // Datas for cyclic mode only
        int16_t  target_torque_;          //name_0x6071_00  // in "per mile" RT
        int32_t  position_offset_;        //name_0x60B0_00  // in qc
        int32_t  velocity_offset_;        //name_0x60B1_00  // in rpm
        int16_t  torque_offset_;          //name_0x60B2_00  // in "per mile" RT
        uint16_t touch_probe_funct_;      //name_0x60B8_00

        //Datas for profile mode
        uint32_t profile_acceleration_;   //name_0x6083_00  // in rpm/s
        uint32_t profile_deceleration_;   //name_0x6084_00  // in rpm/s
        uint32_t profile_velocity_;       //name_0x6081_00  // in rpm

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


        // bool is_switch_on_;          //bit 0
        // bool is_voltage_enable_;     //bit 1
        // bool is_quick_stop_;         //bit 2
        // bool is_operation_enable_;   //bit 3
        // bool ask_fault_reset_;       //bit 7 //have to set false in normal used

        // //used in profile modes
        // // In PPM contorl mode
        // bool new_setpoint_;          //bit 4 1-> assume || 0-> not assume : target position
        // bool change_set_immediatly_; //bit 5 0-> finish actual || interupt actual : positionnng then start next
        // bool target_position_mode_;  //bit 6 target position is : 0-> absolute value || 1-> relative value
        // bool is_halted_;             //bit 8 0-> execute positionning || 1-> stop axle with deceleration profile

        // // In PVM contorl mode
        // bool is_halted_;             //bit 8 0-> execute motion || 1-> stop axle
        // // In HMM contorl mode
        // bool is_homming_active_;     //bit 4 0-> homing mode inactive || 0->1 : start homing mode || 1-> homing mode active
        // bool is_halted_;             //bit 8 0-> execute instruction of is_homming_active_ || 1-> stop axle with homing acceleration
        // // In PVT contorl mode
        // bool is_ip_enable;           //bit 4 0-> interpolated position mode inactive || 1-> interpolated position mode active
        // bool is_halted_;             //bit 8 0-> execute instruction of is_ip_enable || 1-> stop axle with profile deceleration

        static const uint16_t mask_state_device_status_ = 0x417F;

        // Flag to check State of device in statusword
        // static const uint16_t flag_fault_state_ = 0x0108;
        // static const uint16_t flag_fault_react_disable_state_ = 0x010F;
        // static const uint16_t flag_fault_react_enable_state_ = 0x011F;
        // static const uint16_t flag_switch_on_disable_ = 0x0140;
        // static const uint16_t flag_ready_to_switch_on_ = 0x0121;
        // static const uint16_t flag_switched_on_ = 0x0123;
        // static const uint16_t flag_refresh_power_ = 0x4123;
        // static const uint16_t flag_measure_init_ = 0x4133;
        // static const uint16_t flag_operation_enable_ = 0x0137;
        // static const uint16_t flag_quick_stop_activ_ = 0x0117;

        const std::map<int,std::string> device_state_decode_ = {
            {0x0108,"Fault"},
            {0x010F,"Fault reaction active (disable)"},
            {0x011F,"Fault reaction active (enable)"},
            {0x0140,"Switch on disable"},
            {0x0121,"Ready to switch ON"},
            {0x0123,"Switched ON"},
            {0x4123,"Refresh power stage"},
            {0x4133,"Measure init"},
            {0x0137,"Operation enable"},
            {0x0117,"Quick stop activ"} };

        // Digital output datas
        static const int output_number_ = 5;
        std::array<uint16_t, output_number_> digital_output_mapping_ ; // 5 is number of dig output of Epos3
        std::array<uint16_t, output_number_> digital_output_flag_ ; // 5 is number of dig output of Epos3
        uint16_t digital_output_mask_ = 0;

//----------------------------------------------------------------------------//
//             I N P U T S   B U F F E R    D A T A S                         //
//----------------------------------------------------------------------------//

        // Datas present in profil and cyclic mode
        uint16_t status_word_;                //name_0x6041_00
        int32_t  actual_position_;            //name_0x6064_00  // in qc
        int32_t  actual_velocity_;            //name_0x606C_00  // In rpm
        int8_t   operation_mode_;             //name_0x6061_00
        uint16_t digital_input_state_;        //name_0x2071_01

        // Specific datas for cyclic mode
        int16_t  actual_torque_;              //name_0x6077_00  // In "per mile" RT
        uint16_t touch_probe_status_;         //name_0x60B9_00
        int32_t  touch_probe_position_pos_;   //name_0x60BA_00  // in qc ?
        int32_t  touch_probe_position_neg_;   //name_0x60BB_00  // in qc ?

        //Datas for profile mode
        int16_t  actual_current_;               //name 0x6078_00 // in mA
        int16_t  actual_following_error_;       //name 0x20F4_00 // in qc
        uint16_t interpolation_buffer_status_;  //name 0x20C4_01

        // Digital input datas
        static const int input_number_ = 10;
        std::array<uint16_t, input_number_> digital_input_mapping_ ;
        std::array<uint16_t, input_number_> digital_input_flag_ ;
        uint16_t digital_input_mask_ = 0;

        const std::map<int,std::string> control_mode_decode_ = {
            {1,"profile_position_PPM"},
            {3,"profile_velocite_PVM"},
            {7,"interpolated_position_PVT"},
            {6,"homing_HMM"},
            {8,"position_CSP"},
            {9,"velocity_CSV"},
            {10,"torque_CST"} };

        // Analog input datas
        int16_t analog_input1_value_ = 0;   // in mV
        int16_t analog_input2_value_ = 0;   // in mV
        bool is_analog_input1_active_ = false;
        bool is_analog_input2_active_ = false;

        // Analog output datas
        uint16_t analog_output_value_ = 0;   // in mV
        bool is_analog_output_active_ = false;

        static constexpr double rads2rpm_ = 60/(2*M_PI); // var to convert rad/s to rpm
        static constexpr double rpm2rads_ = (2*M_PI)/60; // var to convert rpm to rad/s
        static constexpr double lambda_calculate_average_ = pow(2,-5); //var to calculate average datas

        general_control_mode_t general_control_mode_;

        uint32_t rated_torque_;   //value of rated torque set by epos3 to decode torque in Nm
        uint32_t encoder1_pulse_nb_config_; // in Qc // value of encoder 1 total pulse number by rotation *4.
        double rad2qc_, qc2rad_;

        double past_average_velocity_, past_average_torque_, past_average_current_;

  };
}
