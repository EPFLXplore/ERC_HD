/**
* @file slave.h
* @author Arnaud Meline
* @brief Header file for slave.cpp.
* @date October 2018 12.
*/

#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <vector>

#include <cassert>

#include <stdint.h>
#include <string.h>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  class soem_slave_impl; //PImpl to use EtherCAT soem
  class soem_master_impl; //PImpl to use EtherCAT soem

  class EthercatUnitDevice;

  /** @brief This class define an EtherCAT slave
  *
  * The slave class regroup all datas / informations for an EtherCAT slave device.
  */
  class Slave
  {
    public:

      /***
      * @brief Constructor of Slave class
      *
      * @param [in] unit_dev_ptr is its EthercatUnitDevice pointer.
      */
      Slave(EthercatUnitDevice* unit_dev_ptr);
      ~Slave();


      /***
      * @brief Function to set master context address
      *
      * @param [in] implmaster is master context address.
      */
      void set_Master_Context_Ptr( soem_master_impl* implmaster);

      /***
      * @brief Function to get slave position on EtherCAT master bus.
      *
      * @return slave position on EtherCAT master bus.
      */
      const int& get_Ec_Bus_Position() const;

      /***
      * @brief Function to set slave position on EtherCAT master bus.
      *
      * @param [in] position slave position on EtherCAT master bus.
      */
      void set_Ec_Bus_Position(const int& position);

      /***
      * @brief Function to get EtherCAT AL state.
      *
      * @return slave EtherCAT AL state.
      */
      uint16_t& get_State() const;

      /***
      * @brief Function to set slave EtherCAT AL state.
      *
      * @param [in] state is the slave EtherCAT AL state.
      */
      void set_State(const uint16_t& state);

      /***
      * @brief Function to get EtherCAT station configured address
      *
      * @return EtherCAT station configured address.
      */
      uint16_t& get_Configadr() const;

      /***
      * @brief Function to set EtherCAT station configured address
      *
      * @param [in] configadr is the EtherCAT station configured address.
      */
      void set_Configadr(const uint16_t& configadr);

      /***
      * @brief Function to get EtherCAT station Alias address
      *
      * @return EtherCAT station Alias address.
      */
      uint16_t& get_Aliasadr() const;

      /***
      * @brief Function to set EtherCAT station Alias address
      *
      * @param [in] configadr is the EtherCAT station Alias address.
      */
      void set_Aliasadr(const uint16_t& aliasadr);

      /***
      * @brief Function to get EtherCAT device manufacturer id.
      *
      * @return EtherCAT device manufacturer id.
      */
      const uint32_t& get_Eep_Man() const;

      /***
      * @brief Function to set EtherCAT device manufacturer id.
      *
      * Some time when EEPROM device is not configure, we have to set this value from ESI file.
      *
      * @param [in] eep_man is the device manufacturer id.
      */
      void set_Eep_Man(const uint32_t& eep_man);

      /***
      * @brief Function to get EtherCAT device id.
      *
      * @return the device id.
      */
      const uint32_t& get_Eep_Id() const;

      /***
      * @brief Function to set EtherCAT device id.
      *
      * Some time when EEPROM device is not configure, we have to set this value from ESI file.
      *
      * @param [in] eep_man is the device id.
      */
      void set_Eep_Id(const uint32_t& eep_id);



      // Outputs config from Master to Slave (input for slave)
      /***
      * @brief Function to get pointer in master  IOmap for output datas (from Master to Slave).
      *
      * @return  pointer in master buffer IOmap for output datas (from Master to Slave).
      */
      uint8_t* get_Output_Address_Ptr() const;

      /***
      * @brief Function to set pointer in master buffer IOmap for output datas (from Master to Slave).
      *
      * @param [in] output_address_ptr  pointer in master buffer IOmap for output datas (from Master to Slave).
      */
      void set_Output_Address_Ptr(uint8_t *output_address_ptr);

      /***
      * @brief Function to get size of slave output buffer in bits (from Master to Slave).
      *
      * @return size of slave output buffer in bits (from Master to Slave).
      */
      const uint16_t& get_Output_Size_Bits() const;

      /***
      * @brief Function to set size of slave output buffer in bits (from Master to Slave).
      *
      * @param [in] outputs_size_bits size of slave output buffer in bits (from Master to Slave).
      */
      void set_Output_Size_Bits(const uint16_t& outputs_size_bits);

      /***
      * @brief Function to get size of slave output buffer in bytes (from Master to Slave).
      *
      * @return size of slave output buffer in bytes (from Master to Slave).
      */
      const uint32_t& get_Output_Size_Bytes() const;

      /***
      * @brief Function to set size of slave output buffer in bytes (from Master to Slave).
      *
      * @param [in] outputs_size_bits size of slave output buffer in bytes (from Master to Slave).
      */
      void set_Output_Size_Bytes(const uint16_t& outputs_size_bytes);

      /***
      * @brief Function to get the start bit in the first byte of output buffer.
      *
      * @return the start bit in the first byte of output buffer.
      */
      const uint8_t& get_Output_Start_Bit() const;

      /***
      * @brief Function to set the start bit in the first byte of output buffer.
      *
      * @param [in] output_start_bit is the start bit in the first byte of output buffer.
      */
      void set_Output_Start_Bit(const uint8_t& output_start_bit);


      // inputs config from Slave to Master (output for slave)
      /***
      * @brief Function to get pointer in master IOmap for input datas (from Slave to Master).
      *
      * @return  pointer in master buffer IOmap for input datas (from Slave to Master).
      */
      uint8_t* get_Input_Address_Ptr() const;

      /***
      * @brief Function to set pointer in master buffer IOmap for input datas (from Slave to Master).
      *
      * @param [in] input_address_ptr  pointer in master buffer IOmap for input datas (from Slave to Master).
      */
      void set_Input_Address_Ptr(uint8_t *input_address_ptr); //Pointer to inputs data in Master IOmap

      /***
      * @brief Function to get size of slave input buffer in bits (from Slave to Master).
      *
      * @return size of slave input buffer in bits (from Slave to Master).
      */
      const uint16_t& get_Input_Size_Bits() const;

      /***
      * @brief Function to set size of slave input buffer in bits (from Slave to Master).
      *
      * @param [in] inputs_size_bits size of slave input buffer in bits (from Slave to Master).
      */
      void set_Input_Size_Bits(const uint16_t& inputs_size_bits);

      /***
      * @brief Function to get size of slave input buffer in bytes (from Slave to Master).
      *
      * @return size of slave input buffer in bytes (from Slave to Master).
      */
      const uint32_t& get_Input_Size_Bytes() const;

      /***
      * @brief Function to set size of slave input buffer in bytes (from Slave to Master).
      *
      * @param [in] inputs_size_bits size of slave input buffer in bytes (from Slave to Master).
      */
      void set_Input_Size_Bytes(const uint16_t& inputs_size_bytes);

      /***
      * @brief Function to get the start bit in the first byte of input buffer.
      *
      * @return the start bit in the first byte of input buffer.
      */
      const uint8_t& get_Input_Start_Bit() const;

      /***
      * @brief Function to set the start bit in the first byte of input buffer.
      *
      * @param [in] input_start_bit is the start bit in the first byte of input buffer.
      */
      void set_Input_Start_Bit(const uint8_t& input_start_bit); //start bit in first input byte


      // SyncManager configuration (ec_smt struct)
      /***
      * @brief Function to get a specific SyncMananger start address.
      *
      * @param [in] sm_id desired SyncManager id.
      * @return the selected SyncMananger start address.
      */
      uint16_t get_SM_StartAddr(const int& sm_id) const;

      /***
      * @brief Function to set a specific SyncMananger start address.
      *
      * @param [in] sm_id desired SyncManager id.
      * @param [in] startaddr SyncMananger start address.
      */
      void set_SM_StartAddr(const int& sm_id, const uint16_t& startaddr);

      /***
      * @brief Function to get a specific SyncMananger buffer length.
      *
      * @param [in] sm_id desired SyncManager id.
      * @return the selected SyncMananger buffer length.
      */
      uint16_t get_SM_SMlength(const int& sm_id) const;

      /***
      * @brief Function to set a specific SyncMananger buffer length.
      *
      * @param [in] sm_id desired SyncManager id.
      * @param [in] sm_length SyncMananger buffer length.
      */
      void set_SM_SMlength(const int& sm_id, const uint16_t& sm_length);

      /***
      * @brief Function to get a specific SyncMananger flags.
      *
      * @param [in] sm_id desired SyncManager id.
      * @return the selected SyncMananger flags.
      */
      uint32_t get_SM_SMflag(const int& sm_id) const;

      /***
      * @brief Function to set a specific SyncMananger flags.
      *
      * @param [in] sm_id desired SyncManager id.
      * @param [in] sm_flag SyncMananger flags.
      */
      void set_SM_SMflag(const int& sm_id, const uint32_t& sm_flag);

      /***
      * @brief Function to get a specific SyncMananger type.
      *
      * @param [in] sm_id desired SyncManager id.
      * @return the selected SyncMananger type.
      */
      uint8_t get_SMtype(const int& sm_id) const;

      /***
      * @brief Function to set a specific SyncMananger type.
      *
      * @param [in] sm_id desired SyncManager id.
      * @param [in] sm_type SyncMananger type.
      */
      void set_SMtype(const int& sm_id, const uint8_t& sm_type);

      // FMMU configurations (ec_fmmut) //fmmu_id < EC_MAXFMMU !!
      /***
      * @brief Function to get a specific FMMU logical start buffer address.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return the selected FMMU logical start buffer address.
      */
      uint32_t get_FMMU_LogStart(const int& fmmu_id) const;

      /***
      * @brief Function to set a specific FMMU logical start buffer address.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_start FMMU logical start buffer address.
      */
      void set_FMMU_LogStart(const int& fmmu_id, const uint32_t& logical_start);

      /***
      * @brief Function to get a specific FMMU logical buffer length.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return the selected FMMU logical buffer length.
      */
      uint16_t get_FMMU_LogLength(const int& fmmu_id) const;

      /***
      * @brief Function to set a specific FMMU logical buffer length.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_length FMMU logical buffer length.
      */
      void set_FMMU_LogLength(const int& fmmu_id, const uint16_t& logical_length);

      /***
      * @brief Function to get the start bit in the first byte of a specific FMMU logical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return start bit in the first byte of a specific FMMU logical buffer.
      */
      uint8_t get_FMMU_LogStartbit(const int& fmmu_id) const;

      /***
      * @brief Function to set the start bit in the first byte of a specific FMMU logical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_start_bit start bit in the first byte of a specific FMMU logical buffer.
      */
      void set_FMMU_LogStartbit(const int& fmmu_id, const uint8_t& logical_start_bit);

      /***
      * @brief Function to get the end bit in the last byte of a specific FMMU logical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return the end bit in the last byte of a desired FMMU logical buffer.
      */
      uint8_t get_FMMU_LogEndbit(const int& fmmu_id) const;

      /***
      * @brief Function to set the end bit in the last byte of a specific FMMU logical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_end_bit end bit in the last byte of FMMU logical buffer.
      */
      void set_FMMU_LogEndbit(const int& fmmu_id, const uint8_t& logical_end_bit);

      /***
      * @brief Function to get a specific FMMU physical start buffer address.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return the selected FMMU physical start buffer address.
      */
      uint16_t get_FMMU_PhysStart(const int& fmmu_id) const;

      /***
      * @brief Function to set a specific FMMU physical start buffer address.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_start FMMU physical start buffer address.
      */
      void set_FMMU_PhysStart(const int& fmmu_id, const uint16_t& physical_start);

      /***
      * @brief Function to get the start bit in the first byte of a specific FMMU physical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return start bit in the first byte of a specific FMMU physical buffer.
      */
      uint8_t get_FMMU_PhysStartBit(const int& fmmu_id) const;

      /***
      * @brief Function to set the start bit in the first byte of a specific FMMU physical buffer.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] logical_start_bit start bit in the first byte of a specific FMMU physical buffer.
      */
      void set_FMMU_PhysStartBit(const int& fmmu_id, const uint8_t& physical_start_bit);

      /***
      * @brief Function to get the type of a specific FMMU.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return type of a specific FMMU.
      */
      uint8_t get_FMMU_FMMUtype(const int& fmmu_id) const;

      /***
      * @brief Function to set the type of a specific FMMU.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] fmmu_type type of FMMU.
      */
      void set_FMMU_FMMUtype(const int& fmmu_id, const uint8_t& fmmu_type);

      /***
      * @brief Function to check if a specific FMUU is active.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return 1 if FMMU active, 0 if unactive.
      */
      uint8_t get_FMMU_FMMUactive(const int& fmmu_id) const;

      /***
      * @brief Function to activate a specific FMUU.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] fmmu_active 1 to activate FMMU, 0 to unactive.
      */
      void set_FMMU_FMMUactive(const int& fmmu_id, const uint8_t& fmmu_active);

      /***
      * @brief Function to get number of first FMMU unused.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @return number of first FMMU unused
      */
      const uint8_t& get_FMMUunused() const;

      /***
      * @brief Function to set number of first FMMU unused.
      *
      * @param [in] fmmu_id desired FMMU id.
      * @param [in] FMMUunused number of first FMMU unused
      */
      void set_FMMUunused(const uint8_t& FMMUunused);

      // DC config
      /***
      * @brief Function to check if a slave can use DC (distributed clock).
      *
      * @return 1 if has DC, 0 if haven't
      */
      bool get_Hasdc() const;

      /***
      * @brief Function to set if a slave can use DC (distributed clock).
      *
      * @param [in] hasdc 1 if has DC, 0 if haven't
      */
      void set_Hasdc(bool hasdc);

      /***
      * @brief Function to check if DC (distributed clock) is activate.
      *
      * @return 1 if active, 0 if unactive
      */
      const uint8_t& get_DCactive();

      /***
      * @brief Function to set if DC (distributed clock) is activate.
      *
      * @param [in] dc_active 1 if active, 0 if unactive
      */
      void set_DCactive(const uint8_t& dc_active);

      /***
      * @brief Function to get delay from master to slave to synchro DC (distributed clock).
      *
      * @return delay beetween master and slave
      */
      int32_t& get_Delay() const;

      /***
      * @brief Function to set delay from master to slave to synchro DC (distributed clock).
      *
      * @param [in] delay delay beetween master and slave
      */
      void set_Delay(const int32_t& delay);

      // < 0 if slave config forced.
      /***
      * @brief Function to check if a slave is already configure.
      *
      * @return 0 if don't configure, 1 or more if configured
      */
      const uint16_t& get_Configindex() const;

      /***
      * @brief Function to set if a slave is already configure.
      *
      * @param [in] configindex  0 if don't configure, 1 or more if configured
      */
      void set_Configindex(const uint16_t& configindex);

      // group
      /***
      * @brief Function to get slave's group ID.
      *
      * @return slave's group ID
      */
      const uint8_t& get_Group_Id() const;

      /***
      * @brief Function to set slave's group ID.
      *
      * @param [in] group_id slave's group ID
      */
      void set_Group_Id(const uint8_t& group_id);

      // Slave Name
      /***
      * @brief Function to get slave's name.
      *
      * @return slave's name
      */
      const std::string get_Name() const;

      /***
      * @brief Function to get slave's name.
      *
      * @param [in] name slave's name
      */
      void set_Name(const std::string& name);

      // Size of SM Vector
      /***
      * @brief Function to get the maximum number of SyncManager .
      *
      * @return maximum of SyncManager can be used
      */
      int get_SizeOf_SM() const;

      // Size of FMMU Vector
      /***
      * @brief Function to get the maximum number of FMMU .
      *
      * @return maximum of FMMU can be used
      */
      int get_SizeOf_FMMU() const;

      // Nb of step needed to update all datas
      /***
      * @brief Function used to get the number of run steps
      *
      * @return number of run steps
      */
      int get_Nb_Run_Steps() const;

      /***
      * @brief Function used to launch the pre_function for one specific run step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void pre_Run_Step(int step);

      /***
      * @brief Function used to launch the post_function for one specific run step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void post_Run_Step(int step);

      /***
      * @brief Function used to get the number of init steps
      *
      * @return number of init steps
      */
      int get_Nb_Init_Steps() const;

      /***
      * @brief Function used to launch the pre_function for one specific init step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void pre_Init_Step(int step);

      /***
      * @brief Function used to launch the post_function for one specific init step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void post_Init_Step(int step);

      /***
      * @brief Function used to get the number of end steps
      *
      * @return number of end steps
      */
      int get_Nb_End_Steps() const;

      /***
      * @brief Function used to launch the pre_function for one specific end step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void pre_End_Step(int step);

      /***
      * @brief Function used to launch the post_function for one specific end step
      *
      * @param [in] step is the specific step number who want to launch.
      */
      void post_End_Step(int step);

      /***
      * @brief Function used to get the current step.
      *
      * @return current number step.
      */
      int get_Current_Step() const;

      /***
      * @brief Function used to set the current step.
      *
      * @param[in] step current number step.
      */
      void set_Current_Step(int step);

      /***
      * @brief Function used to increment value of the current step.
      */
      void increment_Current_Step();

      /***
      * @brief Function used to get the time to wait value.
      *
      * This value (used only in Init and End step) set time to wait beetween two EtherCAT frames.
      * @return value of timewait step
      */
      int get_Timewait_Step() const;

      /***
      * @brief Function used to set the time to wait value.
      *
      * This value (used only in Init and End step) set time to wait beetween two EtherCAT frames.
      * @param [in] timewait value of timewait step
      */
      void set_Timewait_Step(int timewait);


      //Update UnitDevice buffers
      /***
      * @brief Function used to update In/Out buffers address
      */
      void update_Device_Buffers();

      // CanOpen over EtherCAT functions
      /***
      * @brief Function used to launch the CanOpen configuration function
      */
      void canopen_Launch_Configuration();

      /***
      * @brief Function used to send a CoE SDO read.
      *
      * @param[in]  index      = Index to write.
      * @param[in]  sub_index  = Subindex to write.
      * @param[in]  buffer_size= Size in bytes of parameter buffer.
      * @param[out] buffer_ptr = Pointer to parameter buffer.
      * @return Workcounter from last slave response
      */
      int canopen_Read_SDO(uint16_t index, uint8_t sub_index, int buffer_size, void* buffer_ptr);

      /***
      * @brief Function used to send a CoE SDO write.
      *
      * @param[in]  index      = Index to write.
      * @param[in]  sub_index  = Subindex to write.
      * @param[in]  buffer_size= Size in bytes of parameter buffer.
      * @param[out] buffer_ptr = Pointer to parameter buffer.
      * @return Workcounter from last slave response
      */
      int canopen_Write_SDO(uint16_t index, uint8_t sub_index, int buffer_size, void* buffer_ptr);

      // Configuration of DC synchro 0 and 1 signal
      /***
      * @brief Function used to get the sync cycle shift for DC sync config.
      *
      * @return cycle shift value in ns
      */
      int32_t get_Sync_Cycle_Shift();

      /***
      * @brief Function used to get the sync0 cycle time for DC sync config.
      *
      * @return sync0 cycle time value in ns
      */
      uint32_t get_Sync0_Cycle_Time();

      /***
      * @brief Function used to check if DC sync0 is used.
      *
      * @return TRUE if used, FALSE if unused
      */
      bool get_Sync0_Is_Used();

      /**
      * @brief Function used to define DC synchro signal 0.
      *
      * @param [in] cycle_time_0 is Cycltime SYNC0 in ns.
      * @param [in] cycle_shift is CyclShift in ns.
      */
      void config_DC_Sync0(uint32_t& cycle_time_0, int32_t& cycle_shift);

      /***
      * @brief Function used to get the sync1 cycle time for DC sync config.
      *
      * @return sync1 cycle time value in ns
      */
      uint32_t get_Sync1_Cycle_Time();

      /***
      * @brief Function used to check if DC sync1 is used.
      *
      * @return TRUE if used, FALSE if unused
      */
      bool get_Sync0_1_Is_Used();

      /**
      * @brief Function used to define DC synchro signal 0 and 1.
      *
      * @param [in] cycle_time_0 is Cycltime SYNC0 in ns.
      * @param [in] cycle_time_1 is Cycltime SYNC1 in ns. This time is a delta time in relation to the SYNC0 fire. If CylcTime1 = 0 then SYNC1 fires a the same time as SYNC0.
      * @param [in] cycle_shift is CyclShift in ns.
      */
      void config_DC_Sync0_1(uint32_t& cycle_time_0, uint32_t& cycle_time_1, int32_t& cycle_shift); // time in ns


    private:
      EthercatUnitDevice* device_;
      soem_slave_impl *implslave_; //PImpl to use EtherCAT soem
      soem_master_impl* master_context_ptr_;

      int ec_bus_pos_; //Position on EtherCAT bus

      int current_step_; // Current step executed by master.
      int timewait_step_; //Time to wait beetween 2 init or end steps (in us)

      // var to activate and configure Distributed clock syncro 0 signal.
      bool dc_sync0_is_used_;
      bool dc_sync0_1_is_used_;
      int32_t dc_sync_cycle_shift_;  // time in ns
      uint32_t dc_sync0_cycle_time_;  // time in ns
      uint32_t dc_sync1_cycle_time_;  // time in ns

  };

}
