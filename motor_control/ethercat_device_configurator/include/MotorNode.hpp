#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/lockfree/queue.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hd_interfaces/msg/motor_commands.hpp"

#include "MotorController.h" // Include your MotorController header
#include "Motor.h"

class MotorNode : public rclcpp::Node
{
public:
    MotorNode(const std::string &config_path);
    std::shared_ptr<MotorController> getMotorController();


private:
    void motor_state_callback();
    void command_callback(const hd_interfaces::msg::MotorCommands::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<hd_interfaces::msg::MotorCommands>::SharedPtr motors_state_publisher_;
    rclcpp::Subscription<hd_interfaces::msg::MotorCommands>::SharedPtr motors_command_subscription_;

    std::shared_ptr<MotorController> motor_controller_;
};

#endif // MOTOR_NODE_HPP
