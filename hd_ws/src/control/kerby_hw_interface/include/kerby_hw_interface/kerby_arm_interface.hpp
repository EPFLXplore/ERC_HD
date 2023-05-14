#ifndef KERBY_ARM_INTERFACE_HPP_
#define KERBY_ARM_INTERFACE_HPP_


#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"



namespace kerby_hw_interface {

class KerbyArmInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(KerbyArmInterface)

    //KerbyArmInterface();

    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    ROS2_KERBY_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    //hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    //hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
    // Store the command for the simulated robot
    std::vector<double> hw_position_commands_;
    std::vector<double> hw_velocity_commands_;
    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
};

}   // kerby_hw_interface


#endif  // KERBY_ARM_INTERFACE_HPP_