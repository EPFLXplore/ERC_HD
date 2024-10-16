#include "kerby_hw_interface/kerby_arm_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace kerby_hw_interface {

hardware_interface::CallbackReturn KerbyArmInterface::on_init(const hardware_interface::HardwareInfo &hardware_info) {
    if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    // initialize all member variables and stuff

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        // Kerby arm has exactly two state interfaces and one command interface on each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("KerbyArmInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("KerbyArmInterface"),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("KerbyArmInterface"),
                "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("KerbyArmInterface"),
                "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("KerbyArmInterface"),
                "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    // setup communication with real motor control code
    init_communication();
    
    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn KerbyArmInterface::on_configure(const rclcpp_lifecycle::State &previous_state) {
    // setup communication to the hardware and set everything up so that the hardware can be activated

    // reset values always when configuring hardware    TODO: see if I need to make this agree with the current states of the arm to prevent **HOMING**
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        hw_position_states_[i] = 0;
        hw_velocity_states_[i] = 0;
        hw_position_commands_[i] = hw_position_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("KerbyArmInterface"), "Successfully configured");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> KerbyArmInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> KerbyArmInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    }

    return command_interfaces;
}


hardware_interface::CallbackReturn KerbyArmInterface::on_activate(const rclcpp_lifecycle::State &previous_state) {
    // enable hardware power

    // command and state should be equal when starting
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        hw_position_commands_[i] = hw_position_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("KerbyArmInterface"), "Successfully activated");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn KerbyArmInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    // the opposite of on_activate
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type KerbyArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // get states from hardware and store them to internal variables defined in export_state_interfaces

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type KerbyArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // command the hardware based onthe values stored in internal varialbes defined in export_command_interfaces

    // sending_commands_ = true;    // TODO: remove this
    if (scanning_ || !sending_commands_) return hardware_interface::return_type::OK;

    std_msgs::msg::Float64MultiArray msg;
    for (uint i = 0; i < hw_position_states_.size(); i++) {
        msg.data.push_back(hw_position_commands_[i]);
    }
    hd_cmd_pub_->publish(msg);

    return hardware_interface::return_type::OK;
}


/*
hardware_interface::CallbackReturn KerbyArmInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    // the opposite of on_configure
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KerbyArmInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    // shutdown hardware gracefully
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KerbyArmInterface::on_error(const rclcpp_lifecycle::State &previous_state) {
    // handle diferent errors
    return CallbackReturn::SUCCESS;
}
*/

void KerbyArmInterface::communication_spin() {
    rclcpp::spin(communication_node_);
}

void KerbyArmInterface::init_communication() {
    communication_node_ = std::make_shared<rclcpp::Node>("hardware_arm_interface_communication_node");

    hd_cmd_pub_ = communication_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_pos_cmd", 10);
    hd_state_sub_ = communication_node_->create_subscription<sensor_msgs::msg::JointState>("/HD/motor_control/joint_telemetry", 10,
        std::bind(&KerbyArmInterface::arm_state_callback, this, std::placeholders::_1));
    mode_change_sub_ = communication_node_->create_subscription<std_msgs::msg::Int8>("/HD/fsm/mode_change", 10,
        std::bind(&KerbyArmInterface::mode_change_callback, this, std::placeholders::_1));
    position_mode_switch_ = communication_node_->create_subscription<std_msgs::msg::Int8>("/HD/kinematics/position_mode_switch", 10,
        std::bind(&KerbyArmInterface::position_mode_switch_callback, this, std::placeholders::_1));
    
    std::thread first(&KerbyArmInterface::communication_spin, this);
    first.detach();

}

// void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
//     RCLCPP_INFO(this->get_logger(), "Logging JointState message:");
//     RCLCPP_INFO(this->get_logger(), "  Name: %s", format_vector(msg->name).c_str());
//     RCLCPP_INFO(this->get_logger(), "  Position: %s", format_vector(msg->position).c_str());
//     RCLCPP_INFO(this->get_logger(), "  Velocity: %s", format_vector(msg->velocity).c_str());
//     RCLCPP_INFO(this->get_logger(), "  Effort: %s", format_vector(msg->effort).c_str());
// }

template<typename T>
std::string format_vector(const std::vector<T> &vec) {
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i)
    {
        oss << vec[i];
        if (i != vec.size() - 1)
        {
            oss << ", ";
        }
    }
    return oss.str();
}

void KerbyArmInterface::arm_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // RCLCPP_INFO(rclcpp::get_logger("KerbyArmInterface"), "  Position: %s", format_vector(msg->position).c_str());
    uint size = (hw_position_states_.size() < msg->position.size()) ? hw_position_states_.size() : msg->position.size();
    uint limit = size >= 6 ? 6 : size;
    for (uint i = 0; i < limit; i++) {
        hw_position_states_[i] = msg->position[i];
        hw_velocity_states_[i] = msg->velocity[i];
        if (scanning_ || !sending_commands_) {
            hw_position_commands_[i] = msg->position[i];
        }
    }
    if (size < hw_position_states_.size()) {
        for (uint i = size; i < hw_position_states_.size(); i++) {
            hw_position_states_[i] = msg->position[i];
            hw_velocity_states_[i] = msg->velocity[i];
            if (scanning_ || !sending_commands_) {
                hw_position_commands_[i] = msg->position[i];
            }
        }
    }

    scanning_ = false;
}

void KerbyArmInterface::mode_change_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    static int IDLE = -1;
    static int MANUAL_INVERSE = 0;
    static int MANUAL_DIRECT = 1;
    static int SEMI_AUTONOMOUS = 2;
    static int AUTONOMOUS = 3;
    int mode = msg->data;
    sending_commands_ = false;
    //if (mode != MANUAL_DIRECT && mode != IDLE) sending_commands_ = false;   // TODO: maybe always put it to false (if manual direct mode, we don't want to move through here anyway)
}

void KerbyArmInterface::position_mode_switch_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    if (msg->data == 1) {
        sending_commands_ = true;
        RCLCPP_INFO(rclcpp::get_logger("KerbyArmInterface"), "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    }
}



}   // kerby_hw_interface


PLUGINLIB_EXPORT_CLASS(kerby_hw_interface::KerbyArmInterface, hardware_interface::SystemInterface)