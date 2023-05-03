#include "trajectory_planner/planner.h"


Planner::Planner(rclcpp::Node::SharedPtr node) : 
    m_move_group(node, m_planning_group),
    m_planning_scene_interface(),
    //m_joint_model_group(m_move_group.getCurrentState()->getJointModelGroup(m_planning_group)),
    m_logger(node->get_logger())
{
    auto pose_target_sub = node->create_subscription<geometry_msgs::msg::Pose>("/kinematics/pose_goal", 10, std::bind(&Planner::poseTargetCallback, this, _1));
    auto joint_target_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>("/kinematics/joint_goal", 10, std::bind(&Planner::jointTargetCallback, this, _1));

    setScalingFactors(1, 1);
}

Planner::TrajectoryStatus Planner::reachTargetPose(const geometry_msgs::msg::Pose &target) {
    m_move_group.setPoseTarget(target);
    if (!plan()) return Planner::TrajectoryStatus::PLANNING_ERROR;
    if (!execute()) return Planner::TrajectoryStatus::EXECUTION_ERROR;
    return Planner::TrajectoryStatus::SUCCESS;
}

Planner::TrajectoryStatus Planner::reachTargetJointValues(const std::vector<double> &target) {
    m_move_group.setJointValueTarget(target);
    if (!plan()) return Planner::TrajectoryStatus::PLANNING_ERROR;
    if (!execute()) return Planner::TrajectoryStatus::EXECUTION_ERROR;
    return Planner::TrajectoryStatus::SUCCESS;
}

bool Planner::plan() {
    return (m_move_group.plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::execute() {
    return (m_move_group.execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void Planner::setScalingFactors(double vel, double accel) {
    m_move_group.setMaxVelocityScalingFactor(vel);
    m_move_group.setMaxAccelerationScalingFactor(accel);
}

void Planner::poseTargetCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    RCLCPP_INFO(m_logger, "Received pose goal");
    reachTargetPose(*msg);
}

void Planner::jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    RCLCPP_INFO(m_logger, "Received joint goal");
    reachTargetJointValues(msg->data);
}

