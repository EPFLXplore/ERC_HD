#include "rclcpp/rclcpp.hpp"
#include "trajectory_planner/planner.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto planner = std::make_shared<Planner>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(planner);
    std::thread([&executor]() { executor.spin(); }).detach();

    planner->config();

    planner->loop();

    rclcpp::shutdown();
    return 0;
}