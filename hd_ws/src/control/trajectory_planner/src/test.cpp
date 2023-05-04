#include <unistd.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_planner");

    auto pose_target_pub = node->create_publisher<geometry_msgs::msg::Pose>("/kinematics/pose_goal", 10);
    auto joint_target_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/kinematics/joint_goal", 10);

    geometry_msgs::msg::Pose msg1;
    msg1.orientation.w = 1.0;
    msg1.position.x = 0.2;
    msg1.position.y = 0.0;
    msg1.position.z = 0.8;

    pose_target_pub->publish(msg1);
    
    return 0;
}
