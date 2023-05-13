#include "kerby_hw_interface/state_keeper.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


rclcpp::Node::SharedPtr node;


namespace state_keeper {
//extern sensor_msgs::msg::JointState STATES;


void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    STATES.position = msg->position;
    STATES.velocity = msg->velocity;
    STATES.effort = msg->effort;
}

void config() {
    for (int i=0; i<8; i++) {
        STATES.position.push_back(0);
        STATES.velocity.push_back(0);
        STATES.effort.push_back(0);
    }
    READY = true;
}

}   // state_keeper


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("state_keeper");

    state_keeper::states_sub = node->create_subscription<sensor_msgs::msg::JointState>("/HD/arm_control/joint_telemetry", 10, std::bind(&state_keeper::stateCallback, std::placeholders::_1));

    state_keeper::cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("/HD/kinematics/joint_cmd", 10);

    state_keeper::config();

    //rclcpp::spin(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    while (1) {
        RCLCPP_INFO(node->get_logger(), "%f %f %f %f %f %f",    state_keeper::STATES.position[0], 
                                                                state_keeper::STATES.position[1], 
                                                                state_keeper::STATES.position[2],
                                                                state_keeper::STATES.position[3],
                                                                state_keeper::STATES.position[4],
                                                                state_keeper::STATES.position[5]);
    }

    rclcpp::shutdown();
    return 0;
}