#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "std_msgs/msg/int8.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


class PlannerSupervisor : public rclcpp::Node {
private:
    const std::string                                                   m_planning_group = "kerby_arm_group";
    moveit::planning_interface::MoveGroupInterface*                     m_move_group;
    moveit::planning_interface::PlanningSceneInterface*                 m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                m_joint_model_group;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                m_sanity_feedback_sub;
    std::chrono::time_point<std::chrono::steady_clock>                  m_last_sanity_feedback_time = std::chrono::steady_clock::now();

    void sanityFeedbackCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        m_last_sanity_feedback_time = std::chrono::steady_clock::now();
    }

    void initCommunication() {
        m_sanity_feedback_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/kinematics/planner_sanity_feedback", 10, std::bind(&PlannerSupervisor::sanityFeedbackCallback, this, _1));
    }

    bool sanityFeedbackOld() {
        static const double command_expiration = 1000;
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now-m_last_sanity_feedback_time).count();
        if (dt > command_expiration) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory planner has died, time : %f", dt);
            return true;
        }
        return false;
    }


public:

    PlannerSupervisor(rclcpp::NodeOptions node_options) : Node("kinematics_trajectory_planner_supervisor", node_options)
    {}

    void config()
    {
        m_move_group = new moveit::planning_interface::MoveGroupInterface(shared_from_this(), m_planning_group);
        m_planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
        m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);

        initCommunication();
    }

    void loop() {
        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            if (sanityFeedbackOld()) {
                m_move_group->stop();
                //RCLCPP_ERROR(this->get_logger(), "Trajectory planner has died");
            }
            rate.sleep();
        }
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto supervisor = std::make_shared<PlannerSupervisor>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(supervisor);
    std::thread([&executor]() { executor.spin(); }).detach();

    supervisor->config();

    supervisor->loop();

    rclcpp::shutdown();
    return 0;
}
