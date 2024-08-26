#include "MotorNode.hpp"

MotorNode::MotorNode(const std::string &config_path)
    : Node("motor_node")
{
    RCLCPP_DEBUG(this->get_logger(), "Building Motor Node");

    motors_state_publisher_ = this->create_publisher<hd_interfaces::msg::MotorCommands>("motor_status", 10);

    motors_command_subscription_ = this->create_subscription<hd_interfaces::msg::MotorCommands>(
        "motor_command",
        10,
        std::bind(&MotorNode::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotorNode::motor_state_callback, this));

    motor_controller_ = std::make_unique<MotorController>(config_path);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Loaded Motors\n" << *motor_controller_ << std::endl);

    RCLCPP_DEBUG(this->get_logger(), "Motor Node Built");

}

void MotorNode::motor_state_callback()
{
    RCLCPP_DEBUG(this->get_logger(), "Publishing motor state");
    hd_interfaces::msg::MotorCommands msg;
    msg.header.stamp = this->get_clock()->now();

    for (const auto &motor : motor_controller_->getMotors())
    {
        // auto motor = motor.get();
        msg.name.push_back(motor.get().getName());
        msg.position.push_back(motor.get().getReadPosition()); // or handle the lack of value as needed
        msg.velocity.push_back(motor.get().getReadVelocity()); // or handle the lack of value as needed
        msg.torque.push_back(motor.get().getReadTorque()); // or handle the lack of value as needed
    }
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    motors_state_publisher_->publish(msg);
}

void MotorNode::command_callback(const hd_interfaces::msg::MotorCommands::SharedPtr msg)
{
        RCLCPP_DEBUG(this->get_logger(), "Recieving Command");
        for (size_t i = 0; i < msg->name.size(); ++i)
        {   
            if (!motor_controller_->containsMotor(msg->name[i]))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Motor with the name '" << msg->name[i] << "' not found. Available motors: " << motor_controller_->getMotorNames());
                continue;
            }
            auto& motor = motor_controller_->getMotor(msg->name[i]);
            if (motor.getName() == msg->name[i])
            {
                if (msg->position.size() > i)
                {
                    motor_controller_->setMotorTargetPosition(i, msg->position[i]);
                }
                if (msg->velocity.size() > i)
                {
                    motor_controller_->setMotorTargetVelocity(i, msg->velocity[i]);
                }
                if (msg->torque.size() > i)
                {
                    motor_controller_->setMotorTargetTorque(i, msg->torque[i]);
                }
            }
        }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Recieving Command motor targets: " << *motor_controller_ << std::endl);

    // command_queue_.push(msg->data); // Push the command to the lock-free queue
}

std::shared_ptr<MotorController> MotorNode::getMotorController()
{
    return motor_controller_;
}