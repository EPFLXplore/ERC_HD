#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "kerby_interfaces/msg/pose_goal.hpp"
#include "kerby_interfaces/msg/object.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;


class Planner : public rclcpp::Node {
public:
    enum TrajectoryStatus {
        SUCCESS,
        PLANNING_ERROR,
        EXECUTION_ERROR,
        CANNOT_ATTEMPT
    };

    enum CommandMode {
        MANUAL_INVERSE = 0,
        MANUAL_DIRECT = 1,
        SEMI_AUTONOMOUS = 2,
        AUTONOMOUS = 3
    };

    Planner(rclcpp::NodeOptions node_options);

    ~Planner() {    // TODO: make the pointer shared so no need for destructor
        delete m_move_group;
        delete m_planning_scene_interface;
    }

    // configuration and workflow
    void config();
    void initCommunication();
    void spin();
    void loop();

    // movement
    TrajectoryStatus reachTargetPose(const geometry_msgs::msg::Pose &target, double velocity_scaling_factor);
    TrajectoryStatus reachTargetJointValues(const std::vector<double> &target);
    TrajectoryStatus reachNamedTarget(const std::string &target);
    TrajectoryStatus computeCartesianPath(std::vector<geometry_msgs::msg::Pose> &waypoints);
    TrajectoryStatus computeCartesianPath(std::vector<geometry_msgs::msg::Pose> &waypoints, double velocity_scaling_factor);
    TrajectoryStatus reachTargetPoseCartesian(const geometry_msgs::msg::Pose &target, double velocity_scaling_factor);
    TrajectoryStatus advanceAlongAxis();

    // planning and execution
    bool plan();
    bool execute();
    bool executeSilent();
    bool execute(moveit_msgs::msg::RobotTrajectory &trajectory);
    TrajectoryStatus planAndExecute();

    void enforceCurrentState();

    void setScalingFactors(double vel, double accel);

    void addBoxToWorld(const std::vector<double> &dim, const geometry_msgs::msg::Pose &pose, std::string &name);

    void updateCurrentPosition();

    bool manualInverseCommandOld();

    void stop();

private:
    void poseTargetCallback(const kerby_interfaces::msg::PoseGoal::SharedPtr msg);

    void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void manualInverseAxisCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void addObjectCallback(const kerby_interfaces::msg::Object::SharedPtr msg);

    void modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg);

    void namedTargetCallback(const std_msgs::msg::String::SharedPtr msg);

    void publishEEFPose();

    void sendTrajFeedback(Planner::TrajectoryStatus status);

    bool canMove();


    const std::string                                                   m_planning_group = "kerby_arm_group";
    moveit::planning_interface::MoveGroupInterface*                     m_move_group;
    moveit::planning_interface::PlanningSceneInterface*                 m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                m_joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan                m_plan;
    rclcpp::Subscription<kerby_interfaces::msg::PoseGoal>::SharedPtr    m_pose_target_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   m_joint_target_sub;
    rclcpp::Subscription<kerby_interfaces::msg::Object>::SharedPtr      m_add_object_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                m_mode_change_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   m_man_inv_axis_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              m_named_target_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr              m_eef_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   m_traj_feedback_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr                   m_position_mode_switch_pub;
    CommandMode                                                         m_mode = CommandMode::MANUAL_DIRECT;
    std::chrono::time_point<std::chrono::steady_clock>                  m_last_man_inv_cmd_time = std::chrono::steady_clock::now();
    bool                                                                m_executing_man_inv_cmd = false;
    std::vector<double>                                                 m_man_inv_axis = {0.0, 0.0, 0.0};
    bool                                                                m_mode_transition_ready = true;
    bool                                                                m_is_executing_path = false;
};
