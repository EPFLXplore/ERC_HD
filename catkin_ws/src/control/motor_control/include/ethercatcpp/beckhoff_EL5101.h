/**
* @file beckhoff_EL5101.h
* @author Arnaud Meline
* @brief EtherCAT driver for beckhoff EL5101 device.
* @date October 2018 12.
* @example beckhoff_EL5101_example.cpp
*/

#pragma once

#include <ethercatcpp/ethercat_unit_device.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class describe the EtherCAT driver for a beckhoff EL5101 device
  *
  * This driver permit to communicate with a "beckhoff EL5101" through an EtherCAT bus.
  * The EL5101 EtherCAT Terminal is an interface for the direct connection of incremental encoders with differential inputs.
  */
  class EL5101 : public EthercatUnitDevice
  {
      public:

        /**
        * @brief Constructor of EL5101 class
        */
        EL5101();
        ~EL5101() = default;

        //! This enum define open circuit detection pin.
        typedef enum
        {
          detection_pin_A,    //!< Open circuit detection on Pin A
          detection_pin_B,    //!< Open circuit detection on Pin B
          detection_pin_C     //!< Open circuit detection on Pin C
        }open_circuit_pin_t;

        //! This enum define latch pin.
        typedef enum
        {
          latch_pin_C,          //!< Latch activation on pin C
          latch_pin_ext_pos,    //!< Latch activation on external pin on positive edge
          latch_pin_ext_neg     //!< Latch activation on external pin on negative edge
        }latch_activation_pin_t;


        /**
        * @brief Function used to enable Latch on C or ext. input
        *
        * @param [in] pin desired pin to latch on (choose in latch_activation_pin_t)
        * @param [in] state TRUE to set, FALSE to disable
        */
        void enable_Latch_On(latch_activation_pin_t pin, bool state);

        /**
        * @brief Function used to enable the counter value
        *
        * When flag is set, the offset value is send to the device
        * @param [in] state TRUE to set
        */
        void enable_Counter_offset(bool state);

        /**
        * @brief Function used to set counter offset value
        *
        * @param [in] value value desired for counter offset
        */
        void set_Counter_Offset_Value(uint32_t value);

        /**
        * @brief Function used to check latch validity
        *
        * The counter value was locked with the "C" or "extern" input.
        * @param [in] pin desired pin to check latch (choose in latch_activation_pin_t)
        * @return TRUE if set, FALSE if disable
        */
        bool check_Latch_Validity_On(latch_activation_pin_t pin);

        /**
        * @brief Function used to check if counter offset was set
        *
        * @return TRUE if set, FALSE if not set
        */
        bool check_Counter_Offset_Set();

        /**
        * @brief Function used to check coutner underflow
        *
        * @return TRUE if underflow, FALSE if not
        */
        bool check_Counter_Underflow();

        /**
        * @brief Function used to check coutner overflow
        *
        * @return TRUE if overflow, FALSE if not
        */
        bool check_Counter_Overflow();

        /**
        * @brief Function used to check an open circuit
        *
        * Indicate an open circuit.
        * @return TRUE if open, FALSE if close
        */
        bool check_Open_Circuit();

        /**
        * @brief Function used to check an extrapolation stall
        *
        * The extrapolated part of the counter is invalid
        * @return TRUE if invalid, FALSE if valid
        */
        bool check_Extrapolation_Stall();

        /**
        * @brief Function used to check state of "alarm" input 1
        *
        * @return input state
        */
        bool get_State_Input_1();

        /**
        * @brief Function used to check state of input A
        *
        * @return input state
        */
        bool get_State_Input_A();

        /**
        * @brief Function used to check state of input B
        *
        * @return input state
        */
        bool get_State_Input_B();

        /**
        * @brief Function used to check state of input C
        *
        * @return input state
        */
        bool get_State_Input_C();

        /**
        * @brief Function used to check state of input gate
        *
        * @return input state
        */
        bool get_State_Input_Gate();

        /**
        * @brief Function used to check state of the extern latch input
        *
        * @return input state
        */
        bool get_State_Input_Ext_Latch();

        /**
        * @brief Function used to check a sync error
        *
        * @return TRUE if invalid, FALSE no error.
        */
        bool check_Sync_Error();

        /**
        * @brief Function used to check validity of datas
        *
        * @return TRUE if invalid, FALSE valid.
        */
        bool check_Data_Validity();

        /**
        * @brief Function used to check datas are updated
        *
        * @return toggle when updated datas
        */
        bool check_Data_Updated();

        /**
        * @brief Function used to get the counter value
        *
        * @return counter value
        */
        uint32_t get_counter_value();

        /**
        * @brief Function used to get the latch value
        *
        * @return latch value
        */
        uint32_t get_Latch_value();

        /**
        * @brief Function used to get the period value
        *
        * @return period value
        */
        uint32_t get_Period_value();

        /**
        * @brief Function used to print all datas available
        */
        void print_All_Datas();

        // Configuration fct never used in cyclic loop
        /**
        * @brief Configuration function used to enable counter reset via the C input
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool enable_C_Reset(bool state);

        /**
        * @brief Configuration function used to trigger a counter reset via the external latch input
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool enable_Ext_Reset(bool state);

        /**
        * @brief Configuration function used to enable an up/down counter
        *
        * Enablement of the up/down counter in place of the encoder with the bit set.
        * Increments are counted at input A. Input B specifies the counting direction.
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool enable_Up_Down_Counter(bool state);

        /**
        * @brief Configuration function used to config the gate polarity
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state "0" to disable, "1" to enable positive gate and "2" to enable negative gate
        * @return TRUE if success, FALSE if fail
        */
        bool config_Gate_Polarity(int state);

        /**
        * @brief Configuration function used to enable input filter
        *
        * Input filter is activated only on A, /A, B, /B, C and /C inputs.
        * When filter is activated, a signal edge must be present for at least
        * 2.4 micro ses in order to be counted as an increment.
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool enable_Input_Filter(bool state);

        /**
        * @brief Configuration function used enable checking open circuit detection on inputs
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] pin desired pin used (choose in open_circuit_pin_t )
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool enable_Open_Circuit_Detection_On(open_circuit_pin_t pin, bool state);

        /**
        * @brief Configuration function used to activate the rotation reversion
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to activate, FALSE to disable
        * @return TRUE if success, FALSE if fail
        */
        bool activate_Rotation_Reversion(bool state);

        /**
        * @brief Configuration function used to change extern reset polarity
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] state TRUE to Rising edge, FALSE to falling edge
        * @return TRUE if success, FALSE if fail
        */
        bool enable_Extern_Reset_Polarity(bool state);


        /**
        * @brief Configuration function used to set the frequency windows
        *
        * This is the minimum time over which the frequency is determined.
        * Default 10 ms [resolution: 1 micro sec]
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired
        * @return TRUE if success, FALSE if fail
        */
        bool set_Frequency_Windows(uint16_t value);

        /**
        * @brief Configuration function used to set the frequency scaling
        *
        * Scaling of the frequency measurement (must be divided by this value
        * to obtain the unit in Hz). If value = 100: "0.01 Hz"
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired
        * @return TRUE if success, FALSE if fail
        */
        bool set_Frequency_Scaling(uint16_t value);

        /**
        * @brief Configuration function used to set the period scaling
        *
        * Resolution of the period in the process data.
        * If value = 100 => "100 ns" period value is a multiple of 100 ns
        * If value = 500 => "500 ns" period value is a multiple of 500 ns
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired
        * @return TRUE if success, FALSE if fail
        */
        bool set_Period_Scaling(uint16_t value);

        /**
        * @brief Configuration function used to set the frequency resolution
        *
        * Resolution of the frequency measurement. If value = 100 => "0.01 Hz"
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired
        * @return TRUE if success, FALSE if fail
        */
        bool set_Frequency_Resolution(uint16_t value);

        /**
        * @brief Configuration function used to set the period resolution
        *
        * Internal resolution of the period measurement.
        * If value = 100 => "100 ns" period value is a multiple of 100 ns.
        * The maximum measurable period can then be approx. 1.6 seconds.
        * If value = 500 => "500 ns" period value is a multiple of 500 ns
        * The maximum measurable period can then be approx. 32.7 ms.
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired
        * @return TRUE if success, FALSE if fail
        */
        bool set_Period_Resolution(uint16_t value);


        /**
        * @brief Configuration function used to set the frequency wai time
        *
        * Wainting time for the frequency measurement in ms.
        *
        * Never use a configuration function in the cyclic loop !
        * @param [in] value value desired in ms
        * @return TRUE if success, FALSE if fail
        */
        bool set_Frequency_Wait_Time(uint16_t value);

      private:

        void update_Command_Buffer();
        void unpack_Status_Buffer();




//----------------------------------------------------------------------------//
//                M A I L B O X    D E F I N I T I O N S                      //
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
          uint8_t command_word = 0; //name_7010_01__04;
          uint8_t empty_5 = 0;
          uint32_t set_counter_value = 0;   //name_7010_11;
        } __attribute__((packed)) buffer_out_cyclic_command_t;
        #pragma pack(pop)

        #pragma pack(push, 1)
        typedef struct buffer_in_cyclic_status
        {
          uint8_t status_word_1;          //name_6010_01__08;
          uint8_t status_word_2;          //name_6010_09__10;
          uint32_t counter_value;         //name_6010_11;
          uint32_t latch_value;           //name_6010_12;
          uint32_t period_value;          //name_6010_14;
        } __attribute__((packed)) buffer_in_cyclic_status_t;
        #pragma pack(pop)

        // Command variable
        uint8_t command_word_;
        uint32_t set_counter_value_;

        //Status variable
        uint8_t status_word_1_;
        uint8_t status_word_2_;
        uint32_t counter_value_;
        uint32_t latch_value_;
        uint32_t period_value_;
  };
}
