#ifndef STATE_KEEPER_HPP_
#define STATE_KEEPER_HPP_


#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>


class StateKeeper {
    public:
        static bool m_ready;
        static sensor_msgs::msg::JointState m_states;

        static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_sub;
        static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub;

        static void config() {
            for (int i=0; i<10; i++) {
                m_states.position.push_back(0);
                m_states.velocity.push_back(0);
                m_states.effort.push_back(0);
            }
            m_ready = true;
        }

        static bool ready() { return m_ready; }

        static void write(std::vector<double> position) {
            //if (!m_ready) return;

            m_states.position = position;
            // sensor_msgs::msg::JointState msg;
            // msg.position = position;
            // cmd_pub->publish(msg);
        }

        static void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            m_states.position = msg->position;
            m_states.velocity = msg->velocity;
            m_states.effort = msg->effort;
        }
};

bool StateKeeper::m_ready = false;
sensor_msgs::msg::JointState StateKeeper::m_states;// = sensor_msgs::msg::JointState();


#endif  // STATE_KEEPER_HPP_