#include "rclcpp/rclcpp.hpp"
#include "trajectory_planner/planner.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto planner = std::make_shared<Planner>(node_options);
    
    //rclcpp::spin(planner);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(planner);
    std::thread([&executor]() { executor.spin(); }).detach();

    planner->config();

    planner->spin2();

    // std::thread first(&Planner::spin1, planner);
    // std::thread second(&Planner::spin2, planner);

    // first.join();
    // second.join();

    rclcpp::shutdown();
    return 0;
}