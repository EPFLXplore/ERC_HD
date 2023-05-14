#include <unistd.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kerby_interfaces/msg/pose_goal.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_planner");

    auto pose_target_pub = node->create_publisher<kerby_interfaces::msg::PoseGoal>("/HD/kinematics/pose_goal", 10);
    auto joint_target_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_goal", 10);

    kerby_interfaces::msg::PoseGoal msg;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.2;
    pose.position.y = 0.0;
    pose.position.z = 0.8;

    msg.goal = pose;
    msg.cartesian = false;

    pose_target_pub->publish(msg);
    
    return 0;
}
