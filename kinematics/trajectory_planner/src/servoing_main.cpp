#include "rclcpp/rclcpp.hpp"
#include "trajectory_planner/servoing.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto servo_planner = std::make_shared<ServoPlanner>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(servo_planner);
    std::thread([&executor]() { executor.spin(); }).detach();

    servo_planner->config();

    servo_planner->loop();

    rclcpp::shutdown();
    return 0;
}