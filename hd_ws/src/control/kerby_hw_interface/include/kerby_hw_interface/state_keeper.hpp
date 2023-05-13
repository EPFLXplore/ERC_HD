#ifndef STATE_KEEPER_HPP_
#define STATE_KEEPER_HPP_


#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>


namespace state_keeper {

rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_sub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub;

sensor_msgs::msg::JointState STATES;

bool READY = false;

void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

void config();

//void write(std::vector<double> position, std::vector<double> velocity);

void write(std::vector<double> position) {
    //if (!READY) return;

    STATES.position = position;
    // sensor_msgs::msg::JointState msg;
    // msg.position = position;
    // cmd_pub->publish(msg);
}

void dummyWrite() {}


}   // state_keeper


#endif  // STATE_KEEPER_HPP_