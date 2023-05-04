#include "rclcpp/rclcpp.hpp"
#include "trajectory_planner/planner.h"



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto planner = std::make_shared<Planner>(node_options);
    planner->config();
    
    rclcpp::spin(planner);

    rclcpp::shutdown();
    return 0;
}