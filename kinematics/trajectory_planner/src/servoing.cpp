#include "trajectory_planner/servoing.h"
#include "trajectory_planner/quaternion_arithmetic.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.h>


ServoPlanner::ServoPlanner(rclcpp::NodeOptions node_options) : Node("traj_planner_servoing", node_options)
{}

void ServoPlanner::config()
{
    m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), m_planning_group);
    m_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);
    for (size_t i=0; i < m_joint_count; i++) {
        m_joint_goal.target_state.push_back(0.0);
    }
    for (size_t i=0; i < m_joint_count; i++) {
        m_joint_goal.initial_state.push_back(0.0);
    }

    createROSInterfaces();
}

void ServoPlanner::createROSInterfaces() {
    m_mode_change_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/fsm/mode_change", 10, std::bind(&ServoPlanner::modeChangeCallback, this, _1));
    m_direction_torque_sub = this->create_subscription<geometry_msgs::msg::Point>("/HD/sensors/directional_torque", 10, std::bind(&ServoPlanner::directionalTorqueCallback, this, _1));
    m_posistion_command_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_pos_cmd", 10);    
}

void ServoPlanner::modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    ServoPlanner::CommandMode new_mode = static_cast<ServoPlanner::CommandMode>(msg->data);
    m_mode = new_mode;
}

double sq(double x) { return x*x; }

double squaredNorm(const geometry_msgs::msg::Point &vector) {
    return sq(vector.x) + sq(vector.y) + sq(vector.z);
}

double norm(const geometry_msgs::msg::Point &vector) {
    return std::sqrt(squaredNorm(vector));
}

double squaredDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

geometry_msgs::msg::Point pointMul(const geometry_msgs::msg::Point &p, double k) {
    geometry_msgs::msg::Point result;
    result.x = p.x * k;
    result.y = p.y * k;
    result.z = p.z * k;
    return result;
}

geometry_msgs::msg::Point pointAdd(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    geometry_msgs::msg::Point result;
    result.x = p1.x + p2.x;
    result.y = p1.y + p2.y;
    result.z = p1.z + p2.z;
    return result;
}

void normalize(geometry_msgs::msg::Point &vector) {
    double n = norm(vector);
    if (n > 0) {
        vector.x = vector.x/n;
        vector.y = vector.y/n;
        vector.z = vector.z/n;
    }
}

void vectorCopy(const std::vector<double> &origin, std::vector<double> &destination) {
    destination.clear();
    for (auto x: origin) {
        destination.push_back(x);
    }
}

void quaternionCopy(const geometry_msgs::msg::Quaternion &origin, geometry_msgs::msg::Quaternion &destination) {
    destination.x = origin.x;
    destination.y = origin.y;
    destination.z = origin.z;
    destination.w = origin.w;
}

void ServoPlanner::directionalTorqueCallback(const geometry_msgs::msg::Point::SharedPtr torque) {
    static const double treshold_torque = 0.5;
    if (squaredNorm(*torque) < sq(treshold_torque)) return;

    std::thread executor(&ServoPlanner::followDirection, this, *torque);
    executor.detach();
}

void ServoPlanner::followDirection(geometry_msgs::msg::Point direction)
{
    // TODO:
    // increase frequency
    // maybe interpolate between distinct commands
    // maybe (probably) update command at lower rate than sending

    static const double travel_distance = 0.03;  // [m]
    geometry_msgs::msg::Pose current_pose;
    getEEFPose(current_pose);
    double new_goal_treshold_distance = travel_distance * 0.2;
    if (m_joint_goal.is_active && squaredDistance(current_pose.position, m_joint_goal.target_pose.position) > sq(new_goal_treshold_distance)) {
        return;
    }

    normalize(direction);
    geometry_msgs::msg::Point target_point = pointAdd(current_pose.position, pointMul(direction, travel_distance));
    m_joint_goal.target_pose.position = target_point;
    quaternionCopy(current_pose.orientation, m_joint_goal.target_pose.orientation);
    m_joint_goal.target_pose.orientation = current_pose.orientation;

    m_joint_goal.target_state.clear();
    if (!getIK(m_joint_goal.target_pose, m_joint_goal.target_state)) {
        return;
    }
    copyCurrentJointState(m_joint_goal.initial_state);
    static const double execution_speed = 0.05;      // [m/s] in cartesian space
    m_joint_goal.execution_time_seconds = travel_distance * execution_speed;
    m_joint_goal.start();
}

void ServoPlanner::getEEFPose(geometry_msgs::msg::Pose &pose) {
    pose = m_move_group->getCurrentPose().pose;
}

bool ServoPlanner::getIK(const geometry_msgs::msg::Pose &ik_pose, const std::vector<double> &ik_seed_state, std::vector<double> &solution) {
    moveit_msgs::msg::MoveItErrorCodes error_code;
    return m_move_group->getCurrentState()->getRobotModel()->getJointModelGroup(m_planning_group)->getSolverInstance()->getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool ServoPlanner::getIK(const geometry_msgs::msg::Pose &ik_pose, std::vector<double> &solution) {
    const double *positions = m_move_group->getCurrentState()->getVariablePositions();
    std::vector<double> current_state_positions;
    copyCurrentJointState(current_state_positions);
    return getIK(ik_pose, current_state_positions, solution);
}

void ServoPlanner::copyCurrentJointState(std::vector<double> &destination) {
    const double *positions = m_move_group->getCurrentState()->getVariablePositions();
    destination.clear();
    for (size_t i = 0; i < m_joint_count; i++) {
        destination.push_back(positions[i]);
    }
}


void ServoPlanner::spin()
{
    rclcpp::spin(shared_from_this());
}

void ServoPlanner::loop()
{
    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        switch(m_mode) {
            case ServoPlanner::CommandMode::COMPLIANT_MOTION:
                // RCLCPP_INFO(this->get_logger(), "COMPLIANT BEHAVIOUR MODE");
                if (m_joint_goal.is_active) {
                    std_msgs::msg::Float64MultiArray msg;
                    m_joint_goal.getAdvancement(msg);
                    m_posistion_command_pub->publish(msg);
                }
                break;
        }
        rate.sleep();
    }
}
