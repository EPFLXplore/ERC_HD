#ifndef KERBY_ARM_INTERFACE_HPP_
#define KERBY_ARM_INTERFACE_HPP_


#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "visibility_control.h"



namespace kerby_hw_interface {

class KerbyArmInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(KerbyArmInterface)

    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type start() override;

    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type stop() override;
    
    //ROS2_KERBY_HARDWARE_PUBLIC
    //hardware_interface::return_type on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    
    //ROS2_KERBY_HARDWARE_PUBLIC
    //hardware_interface::return_type on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type read() override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type write() override;
    //hardware_interface::return_type on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    //hardware_interface::return_type on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    //hardware_interface::return_type on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
    // Store the command for the simulated robot
    std::vector<double> hw_position_commands_;
    std::vector<double> hw_velocity_commands_;
    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
};

}   // kerby_hw_interface


#endif  // KERBY_ARM_INTERFACE_HPP_