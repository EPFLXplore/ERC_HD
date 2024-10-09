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
    std::vector<double>                                 target_velocities_rads; // velocities that would allow reaching the target state in the requested time
    std::chrono::time_point<std::chrono::steady_clock>  start_time = std::chrono::steady_clock::now();
    double                                              execution_time_seconds;
    const std::vector<double>                           max_velocities_rads = {0.2, 0.13, 0.13, 0.2, 0.2, 0.2};

    void limitTrajectorySpeed() {
        for (int i = 0; i < initial_state.size(); i++) {
            double joint_distance_rad = abs(target_state[i]-initial_state[i]);
            double joint_speed_rads = joint_distance_rad / execution_time_seconds;
            double capped_speed = std::min(joint_speed_rads, max_velocities_rads[i]);
            execution_time_seconds *= joint_speed_rads/capped_speed;
        }
        target_velocities_rads.clear();
        for (int i = 0; i < initial_state.size(); i++) {
            double joint_distance_rad = target_state[i]-initial_state[i];
            double joint_speed_rads = joint_distance_rad / execution_time_seconds;
            target_velocities_rads.push_back(joint_speed_rads);
        }
    }

    void start() {
        limitTrajectorySpeed();
        start_time = std::chrono::steady_clock::now();
        is_active = true;
    }

    void stop() {
        is_active = false;
    }

    double getExecutionRatio() {
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / 1000.0;
        if (elapsed_time >= execution_time_seconds) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("traj_planner_servoing"), "RATIO IS 1");

            stop();
            return 1.0;
        }
        return elapsed_time / execution_time_seconds;
    }

    void getPositionAdvancement(std_msgs::msg::Float64MultiArray &array) {
        double ratio = getExecutionRatio();
        if (ratio == 1.0) {
            array.data = target_state;
            return;
        }
        array.data.clear();
        for (size_t i = 0; i < target_state.size(); i++) {
            array.data.push_back(initial_state[i] + (target_state[i]-initial_state[i])*ratio);
        }
    }

    void getVelocityAdvancement(std_msgs::msg::Float64MultiArray &array) {
        double ratio = getExecutionRatio();
        for (size_t i = 0; i < target_velocities_rads.size(); i++) {
            array.data.push_back(ratio == 1.0 ? 0.0 : target_velocities_rads[i]/max_velocities_rads[i]);
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

    void directionalTorqueCallback2(const geometry_msgs::msg::Point::SharedPtr torque);

    void followDirection(geometry_msgs::msg::Point direction, double travel_distance, double execution_speed, double interpolation_ratio);
    
    void stop();

    void velocityStop();

    void positionStop();

    void publishJointCommand();

    const std::string                                                       m_planning_group = "kerby_arm_group";
    const uint                                                              m_joint_count = 6;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>         m_move_group;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>     m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                    m_joint_model_group;
    JointGoalInfo                                                           m_joint_goal;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                    m_mode_change_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr              m_direction_torque_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr          m_posistion_command_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr          m_velocity_command_pub;
    CommandMode                                                             m_mode = CommandMode::MANUAL_DIRECT;
    bool                                                                    m_velocity_control = true;
};
