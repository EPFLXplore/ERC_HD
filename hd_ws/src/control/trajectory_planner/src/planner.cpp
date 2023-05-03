#include "trajectory_planner/planner.h"


Planner::TrajectoryStatus Planner::reachTargetPose(geometry_msgs::msg::Pose &target) {
    m_move_group.setPoseTarget(target);
    if (!plan()) return Planner::TrajectoryStatus::PLANNING_ERROR;

}

bool Planner::plan() {
    return (move_group.plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::execute() {
    return (move_group.execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}