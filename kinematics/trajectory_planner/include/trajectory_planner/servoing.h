#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "std_msgs/msg/int8.hpp"

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using std::placeholders::_1;


struct JointGoalInfo {
    bool                                                is_active = false;
    std::vector<double>                                 target_state;
    std::vector<double>                                 initial_state;
    geometry_msgs::msg::Pose                            target_pose;    // information not needed to compute joint space trajectory but can be useful
    std::chrono::time_point<std::chrono::steady_clock>  start_time = std::chrono::steady_clock::now();
    double                                              execution_time_seconds;

    void start() {
        start_time = std::chrono::steady_clock::now();
        is_active = true;
    }

    void stop() {
        is_active = false;
    }

    void getAdvancement(std_msgs::msg::Float64MultiArray &array) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now-start_time).count();
        if (elapsed_time >= execution_time_seconds) {
            array.data = target_state;
            stop();
            return;
        }
        double ratio = elapsed_time / execution_time_seconds;
        array.data.clear();
        for (uint i = 0; i < target_state.size(); i++) {
            array.data.push_back(initial_state[i] + (target_state[i]-initial_state[i])*ratio);
        }
    }
};


class ServoPlanner : public rclcpp::Node {
public:
    enum CommandMode {
        IDLE = -1,
        MANUAL_INVERSE = 0,
        MANUAL_DIRECT = 1,
        SEMI_AUTONOMOUS = 2,
        COMPLIANT_MOTION = 3,
    };

    ServoPlanner(rclcpp::NodeOptions node_options);

    ~ServoPlanner() {
    }

    void config();
    void createROSInterfaces();
    void spin();
    void loop();

private:
    bool getIK(const geometry_msgs::msg::Pose &ik_pose, const std::vector<double> &ik_seed_state, std::vector<double> &solution);

    bool getIK(const geometry_msgs::msg::Pose &ik_pose, std::vector<double> &solution);

    void getEEFPose(geometry_msgs::msg::Pose &pose);

    void copyCurrentJointState(std::vector<double> &destination);

    void modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void directionalTorqueCallback(const geometry_msgs::msg::Point::SharedPtr torque);

    void followDirection(geometry_msgs::msg::Point torque);


    const std::string                                                       m_planning_group = "kerby_arm_group";
    const uint                                                              m_joint_count = 6;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>         m_move_group;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>     m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                    m_joint_model_group;
    JointGoalInfo                                                           m_joint_goal;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                    m_mode_change_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr              m_direction_torque_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr          m_posistion_command_pub;
    CommandMode                                                             m_mode = CommandMode::MANUAL_DIRECT;
};
