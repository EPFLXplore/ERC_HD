#include "rclcpp/rclcpp.hpp"
#include "trajectory_planner/planner.h"

rclcpp::Node::SharedPtr node = nullptr;

void callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    //std::cout << "BBBBBBBBBBBBBBBBBBB" << std::endl;
    RCLCPP_INFO(node->get_logger(), "AHASODOAISJDOAJDSA");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::NodeOptions node_options;
    //node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("trajectory_planner");//, node_options);
    Planner planner(node);

    //auto pose_target_sub = node->create_subscription<geometry_msgs::msg::Pose>("/kinematics/pose_goal", 10, std::bind(&callback, _1));

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}