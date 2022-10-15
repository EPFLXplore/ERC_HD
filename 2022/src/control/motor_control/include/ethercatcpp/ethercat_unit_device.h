/**
* @file ethercat_unit_device.h
* @author Arnaud Meline
* @brief Header file for ethercat_unit_device.cpp.
* @date October 2018 12.
*/


#pragma once
#include <ethercatcpp/ethercat_device.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

/*! \namespace ethercatcpp
 *
 * Root namespace for common and general purpose ethercatcpp packages
 */
namespace ethercatcpp {

  /** @brief This class define an EtherCAT unit device
  *
  * A unit device is an EtherCAT device who contain slave informations (it is composed by a dedicated slave object).
  * It define all steps function (run, init and end), define EtherCAT I/O buffers and all EtherCAT slave configurations.
  */
  class EthercatUnitDevice : public EthercatDevice
  {
    public:

      //! This enum define all type of buffers (SyncManager type)
      typedef enum
      {
        ASYNCHROS_OUT = 1,    //!< define an asynchro mailbox out (from master to slave)
        ASYNCHROS_IN  = 2,    //!< define an asynchro mailbox in (from slave to master)
        SYNCHROS_OUT  = 3,    //!< define a synchro buffer out (from master to slave)
        SYNCHROS_IN   = 4     //!< define a synchro buffer in (from slave to master)
      }syncmanager_buffer_t;

      /**
      * @brief Constructor of EthercatUnitDevice class
      */
      EthercatUnitDevice();
      virtual ~EthercatUnitDevice();

      /***
      * @brief Function used to get the "device pointer" vector who compose the device
      *
      * @return a vector of EthercatDevice pointer. This vector contain only its device address (unit device)
      */
      std::vector< EthercatDevice* > get_Device_Vector_Ptr();

      /***
      * @brief Function used to get the device slave address
      *
      * @return the dedicated slave address. Here, in an unit device, a slave address is returned.
      */
      Slave* get_Slave_Address();


      /**
      * @brief Function used to print all slave informations
      *
      */
      void print_Slave_Infos() const ;

      /***
      * @brief Function used to get the number of run steps for the device
      *
      * @return number of run steps
      */
      uint8_t run_Steps();

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
      * @brief Function used to get the number of init steps for the device
      *
      * @return number of init steps
      */
      uint8_t init_Steps();

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
      * @brief Function used to get the number of end steps for the device
      *
      * @return number of end steps
      */
      uint8_t end_Steps();

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
      * @brief Function used to update In/Out buffers address
      */
      void update_Buffers();


      /***
      * @brief Function used to launch the CanOpen configuration function
      */
      void canopen_Launch_Configuration();

      /**
      * @brief Function used to set specific ID to the slave.
      *
      * @param [in] name is slave name.
      * @param [in] manufacturer is the slave manufacturer ID.
      * @param [in] model is the slave model ID.
      */
      void set_Id(const std::string & name, uint32_t manufacturer, uint32_t model);


    protected:

      /**
      * @brief Function used to define a physical buffer (EtherCAT syncManager buffer).
      *
      * This function define a physical buffer and update size of total I/O buffer
      *
      * @tparam T is the data structure type of the buffer.
      * @param [in] type is the type of buffer selected in syncmanager_buffer_t.
      * @param [in] start_addr is the buffer physical start address.
      * @param [in] flags is the specific flag for this buffer.
      */
      template<typename T>
      void define_Physical_Buffer(syncmanager_buffer_t type, uint16_t start_addr, uint32_t flags){

          uint16_t length = (uint16_t)sizeof(T) * 8; //legnth of buffer type in bits !! not in bytes so " * 8"

          switch(type) {
            case ASYNCHROS_OUT :   //Buffer in an asynchro mailbox out (from master to slave)
              //Normaly automaticaly define by device hardware
            break;
            case ASYNCHROS_IN :   //Buffer in an asynchro mailbox in (from slave to master)
              //Normaly automaticaly define by device hardware
            break;
            case SYNCHROS_OUT :  //Buffer is a synchro buffer out (from master to slave)
              buffer_out_by_address_[start_addr]= buffers_out_.size();
              buffers_out_.push_back( std::make_pair( (uint8_t*) nullptr, length ) );
              buffer_length_out_ += length;
            break;
            case SYNCHROS_IN :  //Buffer is a synchro buffer in (from slave to master)
              buffer_in_by_address_[start_addr]= buffers_in_.size();
              buffers_in_.push_back( std::make_pair( (uint8_t*) nullptr, length ) );
              buffer_length_in_ += length;
            break;
          }

          //re-set size of logical buffers with all new definition of a physical buffer
          set_Device_Buffer_Inputs_Sizes(buffer_length_in_);
          set_Device_Buffer_Outputs_Sizes(buffer_length_out_);

          slave_ptr_->set_SM_SMlength(nb_physical_buffer_, length/8); //SM length define in bytes !!
          slave_ptr_->set_SM_StartAddr(nb_physical_buffer_, start_addr);
          slave_ptr_->set_SM_SMflag(nb_physical_buffer_, flags);
          slave_ptr_->set_SMtype(nb_physical_buffer_, (uint8_t)type);
          ++nb_physical_buffer_;
          return;
      }


      /**
      * @brief Function used to get datas of input physical buffer.
      *
      * This function get the raw datas of the input physical buffer and cast them
      * in the data structure type indicate with "template param" to more easyest used.
      *
      * @tparam T is the data structure type of the buffer.
      * @param [in] start_addr is the input buffer physical start address.
      * @return pointer to buffer data with "template param" structure type.
      */
      template<typename T>
      T* get_Input_Buffer(uint16_t start_addr){
          return (reinterpret_cast<T*>(buffers_in_[buffer_in_by_address_.at(start_addr)].first));
      }

      /**
      * @brief Function used to get datas of output physical buffer.
      *
      * This function get the raw datas of the output physical buffer and cast them
      * in the data structure type indicate with "template param" to more easyest used.
      *
      * @tparam T is the data structure type of the buffer.
      * @param [in] start_addr is the output buffer physical start address.
      * @return pointer to buffer data with "template param" structure type.
      */
      template<typename T>
      T* get_Output_Buffer(uint16_t start_addr){
          return (reinterpret_cast<T*>(buffers_out_[buffer_out_by_address_.at(start_addr)].first));
      }

      /**
      * @brief Function used to add a run step and define pre_function and post_function run step.
      *
      * @param [in] pre is the function witch is execute before a run step (generally set commands).
      * @param [in] post is the function witch is execute after a run step (generally get status and datas).
      */
      void add_Run_Step(std::function<void()>&& pre, std::function<void()>&& post);

      /**
      * @brief Function used to add a init step and define pre_function and post_function init step.
      *
      * @param [in] pre is the function witch is execute before a init step (generally set commands).
      * @param [in] post is the function witch is execute after a init step (generally get status and datas).
      */
      void add_Init_Step(std::function<void()>&& pre, std::function<void()>&& post);

      /**
      * @brief Function used to add a end step and define pre_function and post_function end step.
      *
      * @param [in] pre is the function witch is execute before a end step (generally set commands).
      * @param [in] post is the function witch is execute after a end step (generally get status and datas).
      */
      void add_End_Step(std::function<void()>&& pre, std::function<void()>&& post);

      /**
      * @brief Function used to set input buffer size to the slave.
      *
      * @param [in] size is the buffer size in bits.
      */
      void set_Device_Buffer_Inputs_Sizes(uint16_t size);

      /**
      * @brief Function used to set output buffer size to the slave.
      *
      * @param [in] size is the buffer size in bits.
      */
      void set_Device_Buffer_Outputs_Sizes(uint16_t size);

      /**
      * @brief Function used to define if the slave have a distributed clock.
      *
      * @param [in] have_dc state for distributed clock (TRUE if have DC).
      */
      void define_Distributed_clock(bool have_dc);

      /**
      * @brief Function used to define period between two non cyclic step.
      *
      * @param [in] period time desired to wait in us.
      */
      void define_Period_For_No_Cyclic_Step(int period);

      /**
      * @brief Function used to define CanOpen over EtherCAT functions.
      *
      * @param [in] func is the function witch is execute to configure a Canopen slave and its SDO.
      */
      void canopen_Configure_SDO(std::function<void()>&& func);

      /**
      * @brief Function used to send a CoE write SDO packet to the slave
      *
      * @param [in] index is the SDO index to write data.
      * @param [in] sub_index is the SDO sub index to write data.
      * @param [in] value is the data to write.
      *
      * @tparam T represent the data type of input parameter value.
      * @return 0 if communication failed. Worcounter value if success.
      */
      template<typename T>
      int canopen_Write_SDO(uint16_t index, uint8_t sub_index, T &value){
        return (slave_ptr_->canopen_Write_SDO(index, sub_index, (int)sizeof(T), (void*)&value));
      }

      /**
      * @brief Function used to send a CoE read SDO packet to the slave
      *
      * @param [in] index is the SDO index to read data.
      * @param [in] sub_index is the SDO sub index to read data.
      * @param [in] value is the data readed.
      *
      * @tparam T represent the data type of input parameter value.
      * @return 0 if communication failed. Worcounter value if success.
      */
      template<typename T>
      int canopen_Read_SDO(uint16_t index, uint8_t sub_index, T &value){
        return (slave_ptr_->canopen_Read_SDO(index, sub_index, (int)sizeof(T), (void*)&value));
      }

      /**
      * @brief Function used to start the definition of the command PDO mapping.
      *
      * @tparam T represent the type of date register (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_Start_Command_PDO_Mapping(){
        int wkc = 0;
        // Have to desactivate buffer to change it
        T val = 0;
        wkc += this->canopen_Write_SDO(0x1c12, 0x00, val);
        return (wkc);
      }

      /**
      * @brief Function used to add a new command PDO map link.
      *
      * @param [in] pdo_address is the map address who want to add to the PDO.
      * @tparam T represent the type of "date register" (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_Add_Command_PDO_Mapping(uint16_t pdo_address){
        int wkc = 0;
        T val = 0;
        // Check if buffer is desactivate
        wkc += this->canopen_Read_SDO(0x1c12, 0x00, val);
        if (val != 0){ // no in config PDO mode
          return (false);
        }
        // Send new PDO address mapping
        ++nb_command_PDO_mapping_; //increment number of command pdo mapping to write in good index
        wkc += this->canopen_Write_SDO(0x1c12, nb_command_PDO_mapping_, pdo_address);
        if (wkc == 2) {
          return (true);
        }else{
          return (false);
        }
      }

      /**
      * @brief Function used to finish the definition of the command PDO mapping.
      *
      * @tparam T represent the type of date register (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_End_Command_PDO_Mapping(){
        int wkc = 0;
        T val = 0;
        val = (T)nb_command_PDO_mapping_;
        // Have to reactivate buffer with map number to valid it
        wkc += this->canopen_Write_SDO(0x1c12, 0x00, val);
        return (wkc);
      }

      /**
      * @brief Function used to start the definition of the status PDO mapping.
      *
      * @tparam T represent the type of date register (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_Start_Status_PDO_Mapping(){
        int wkc = 0;
        // Have to desactivate buffer to change it
        T val = 0;
        wkc += this->canopen_Write_SDO(0x1c13, 0x00, val);
        return (wkc);
      }

      /**
      * @brief Function used to add a new STATUS PDO map link.
      *
      * @param [in] pdo_address is the map address who want to add to the PDO.
      * @tparam T represent the type of "date register" (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_Add_Status_PDO_Mapping(uint16_t pdo_address){
        int wkc = 0;
        T val = 0;
        // Check if buffer is desactivate
        wkc += this->canopen_Read_SDO(0x1c13, 0x00, val);
        if (val != 0){ // no in config PDO mode
          return (false);
        }
        // Send new PDO address mapping
        ++nb_status_PDO_mapping_; //increment number of Status pdo mapping to write in good index
        wkc += this->canopen_Write_SDO(0x1c13, nb_status_PDO_mapping_, pdo_address);
        if (wkc == 2) {
          return (true);
        }else{
          return (false);
        }
      }

      /**
      * @brief Function used to finish the definition of the status PDO mapping.
      *
      * @tparam T represent the type of date register (generally uint8 or uint16).
      * @return false if communication failed, true if success.
      */
      template<typename T>
      bool canopen_End_Status_PDO_Mapping(){
        int wkc = 0;
        T val = nb_status_PDO_mapping_;
        // Have to reactivate buffer with map number to valid it
        wkc += this->canopen_Write_SDO(0x1c13, 0x00, val);
        return (wkc);
      }

      /**
      * @brief Function used to define DC synchro signal 0.
      *
      * This function active DC sync 0 mode and set timers for a slave.
      *
      * @param [in] cycle_time_0 is Cycltime SYNC0 in ns.
      * @param [in] cycle_shift is CyclShift in ns.
      */
      void config_DC_Sync0(uint32_t cycle_time_0, int32_t cycle_shift);

      /**
      * @brief Function used to define DC synchro signal 0 and 1.
      *
      * This function active DC sync 0 and 1 mode and set timers for a slave.
      *
      * @param [in] cycle_time_0 is Cycltime SYNC0 in ns.
      * @param [in] cycle_time_1 is Cycltime SYNC1 in ns. This time is a delta time in relation to the SYNC0 fire. If CylcTime1 = 0 then SYNC1 fires a the same time as SYNC0.
      * @param [in] cycle_shift is CyclShift in ns.
      */
      void config_DC_Sync0_1(uint32_t cycle_time_0, uint32_t cycle_time_1, int32_t cycle_shift); // time in ns

    private:

      std::function<void()> canopen_config_sdo_;

      //Pointer to the dedicated slave for this device.
      Slave* slave_ptr_;

      std::vector<std::pair<std::function<void()>,std::function<void()>>> run_steps_; //Pair : first = pre , second = post
      std::vector<std::pair<std::function<void()>,std::function<void()>>> init_steps_; //Pair : first = pre , second = post
      std::vector<std::pair<std::function<void()>,std::function<void()>>> end_steps_; //Pair : first = pre , second = post

      int nb_physical_buffer_; // Number of physical buffer
      int buffer_length_in_, buffer_length_out_; //Sum of size of all input/output buffer
      uint8_t nb_command_PDO_mapping_, nb_status_PDO_mapping_; // Used to count nb of PDO declared

      //Input & Output vector
      std::map<uint16_t, uint16_t> buffer_out_by_address_; // key = physical address, value = index of the corresponding elemnt in  buffers_out_
      std::map<uint16_t, uint16_t> buffer_in_by_address_; // key = physical address, value = index of the corresponding elemnt in  buffers_in_
      std::vector<std::pair< uint8_t*, uint16_t >> buffers_out_; // Pair : First = pointer of data struct buffer, second = size of data struct in bits
      std::vector<std::pair< uint8_t*, uint16_t >> buffers_in_; // Pair : First = pointer of data struct buffer, second = size of data struct in bits
  };
} //end of namespace
