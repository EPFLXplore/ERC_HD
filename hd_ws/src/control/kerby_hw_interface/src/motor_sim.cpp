#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;


rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

sensor_msgs::msg::JointState state;


void initState() {
    for (int i=0; i<8; i++) {
        state.position.push_back(0);
        state.velocity.push_back(0);
        state.effort.push_back(0);
    }
}


void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "Received command");
    state.position = msg->position;
    state.velocity = msg->velocity;
}

void publishState() {
    sensor_msgs::msg::JointState msg;
    for (int i=0; i<state.position.size(); i++) {
        msg.position.push_back(state.position[i]);
        msg.velocity.push_back(state.velocity[i]);
        msg.effort.push_back(state.effort[i]);
    }
    state_pub->publish(msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    initState();

    node = std::make_shared<rclcpp::Node>("kinematics_motor_sim");

    state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/HD/arm_control/joint_telemetry", 10);
    cmd_sub = node->create_subscription<sensor_msgs::msg::JointState>("/HD/kinematics/joint_cmd", 10, std::bind(&cmdCallback, std::placeholders::_1));

    auto timer = node->create_wall_timer(10ms, std::bind(&publishState));

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
