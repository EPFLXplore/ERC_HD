#include "kerby_hw_interface/state_keeper.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("state_keeper");

    static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_sub = node->create_subscription<sensor_msgs::msg::JointState>("/HD/arm_control/joint_telemetry", 10, std::bind(&StateKeeper::stateCallback, std::placeholders::_1));
    static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("/HD/kinematics/joint_cmd", 10);

    StateKeeper::config();

    //rclcpp::spin(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    while (1) {
        RCLCPP_INFO(node->get_logger(), "%f %f %f %f %f %f",    StateKeeper::m_states.position[0], 
                                                                StateKeeper::m_states.position[1], 
                                                                StateKeeper::m_states.position[2],
                                                                StateKeeper::m_states.position[3],
                                                                StateKeeper::m_states.position[4],
                                                                StateKeeper::m_states.position[5]);
    }

    rclcpp::shutdown();
    return 0;
}