#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motor_control_interfaces/msg/motor_command.hpp"

#include <iostream>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<motor_control_interfaces::msg::MotorCommand>(
      "motor_command", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const motor_control_interfaces::msg::MotorCommand::SharedPtr msg) const
    {
      std::cout << msg->name << " , " << msg->mode << " , " << msg->command << std::endl;
    }
    rclcpp::Subscription<motor_control_interfaces::msg::MotorCommand>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}