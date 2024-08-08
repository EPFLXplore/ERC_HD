#include "MotorNode.hpp"

MotorNode::MotorNode()
    : Node("motor_node")
{
    motors_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("motor_status", 10);

    motors_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "motor_command",
        10,
        std::bind(&MotorNode::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MotorNode::motor_state_callback, this));
}

void MotorNode::motor_state_callback()
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();

    for (const auto &motor : motor_controller_->getMotors())
    {
        msg.name.push_back(motor.getName());

        auto read_position = motor.getReadPosition();
        if (read_position.has_value())
        {
            msg.position.push_back(read_position.value());
        }
        else
        {
            msg.position.push_back(0.0); // or handle the lack of value as needed
        }

        auto read_velocity = motor.getReadVelocity();
        if (read_velocity.has_value())
        {
            msg.velocity.push_back(read_velocity.value());
        }
        else
        {
            msg.velocity.push_back(0.0); // or handle the lack of value as needed
        }

        auto read_torque = motor.getReadTorque();
        if (read_torque.has_value())
        {
            msg.effort.push_back(read_torque.value());
        }
        else
        {
            msg.effort.push_back(0.0); // or handle the lack of value as needed
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    motors_state_publisher_->publish(msg);
}

void MotorNode::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg->data.c_str());
    // command_queue_.push(msg->data); // Push the command to the lock-free queue
}
