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

void normalize(geometry_msgs::msg::Point &vector) {
    double n = norm(vector);
    if (n > 0) {
        vector.x = vector.x/n;
        vector.y = vector.y/n;
        vector.z = vector.z/n;
    }
}

void ServoPlanner::directionalTorqueCallback(const geometry_msgs::msg::Point::SharedPtr torque) {
    std::thread executor(&ServoPlanner::followDirectionalTorque, this, torque);
    executor.detach();
}

void ServoPlanner::followDirectionalTorque(const geometry_msgs::msg::Point::SharedPtr torque)
{
    // TODO:
    // increase frequency
    // maybe interpolate between distinct commands
    // maybe (probably) update command at lower rate than sending
    double treshold_torque = 0.5;
    if (squaredNorm(*torque) < sq(treshold_torque)) return

    normalize(*torque);
    geometry_msgs::msg::Pose current_pose;
    getEEFPose(current_pose);
    geometry_msgs::msg::Point current_point = current_pose.position;
    geometry_msgs::msg::Point target_point;
    double scaling = 0.005;  // [m]
    target_point.x = current_point.x + torque->x * scaling;
    target_point.y = current_point.y + torque->y * scaling;
    target_point.z = current_point.z + torque->z * scaling;
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = target_point;
    target_pose.orientation = current_pose.orientation;
    std::vector<double> target_joint_config;
    if (getIK(target_pose, target_joint_config)) {
        // send target joint positions to motor control
        std_msgs::msg::Float64MultiArray msg;
        for (auto x: target_joint_config) {
            msg.data.push_back(x);
        }
        m_posistion_command_pub->publish(msg);
    }
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
    static size_t MOTOR_COUNT = 6;
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        current_state_positions.push_back(positions[i]);
    }
    return getIK(ik_pose, current_state_positions, solution);
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
                RCLCPP_INFO(this->get_logger(), "COMPLIANT BEHAVIOUR MODE");
                break;
        }
        rate.sleep();
    }
}
