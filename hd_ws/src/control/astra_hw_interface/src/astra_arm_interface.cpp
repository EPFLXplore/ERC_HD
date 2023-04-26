#include "astra_hw_interface/astra_arm_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace astra_hw_interface {

hardware_interface::return_type AstraArmInterface::configure(const hardware_interface::HardwareInfo &hardware_info) {
    if (configure_default(hardware_info) != hardware_interface::return_type::OK)
        return hardware_interface::return_type::ERROR;
    
    // initialize all member variables and stuff

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        // Astra arm has exactly two state interfaces and one command interface on each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AstraArmInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AstraArmInterface"),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AstraArmInterface"),
                "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AstraArmInterface"),
                "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AstraArmInterface"),
                "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }
    }
    
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type AstraArmInterface::start() {
    // setup communication to the hardware and set everything up so that the hardware can be activated

    // reset values always when configuring hardware    TODO: see if I need to make this agree with the current states of the arm to prevent **HOMING**
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        hw_position_states_[i] = 0;
        hw_velocity_states_[i] = 0;
        hw_position_commands_[i] = 0;
    }

    // command and state should be equal when starting
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        hw_position_commands_[i] = hw_position_states_[i];
    }

    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO(rclcpp::get_logger("AstraArmInterface"), "Successfully configured");

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type AstraArmInterface::stop() {
    return hardware_interface::return_type::OK;
}


std::vector<hardware_interface::StateInterface> AstraArmInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> AstraArmInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    }

    return command_interfaces;
}


hardware_interface::return_type AstraArmInterface::read() {
    // get states from hardware and store them to internal variables defined in export_state_interfaces
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        hw_position_states_[i] = 2*hw_position_commands_[i];
    }

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type AstraArmInterface::write() {
    // command the hardware based onthe values stored in internal varialbes defined in export_command_interfaces

    return hardware_interface::return_type::OK;
}


/*
hardware_interface::CallbackReturn AstraArmInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    // the opposite of on_configure
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AstraArmInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    // shutdown hardware gracefully
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AstraArmInterface::on_error(const rclcpp_lifecycle::State &previous_state) {
    // handle diferent errors
    return CallbackReturn::SUCCESS;
}
*/

}   // astra_hw_interface


PLUGINLIB_EXPORT_CLASS(astra_hw_interface::AstraArmInterface, hardware_interface::SystemInterface)
