#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


class Planner {
    Planner(std::shared_ptr<rclcpp::Node> node) : 
    m_move_group(node, m_planning_group), 
    m_joint_model_group(m_move_group.getCurrentState()->getJointModelGroup(m_planning_group)), 
    m_logger(node->get_logger())
    {}

    Planner::TrajectoryStatus reachTargetPose(geometry_msgs::msg::Pose &target);

    Planner::TrajectoryStatus reachTargetJointValues(std::vector<double> &target);

    bool plan();

    bool execute();


    enum TrajectoryStatus {
        SUCCESS,
        PLANNING_ERROR,
        EXECUTION_ABORTED
    }
private:
    const rclcpp::Logger                                    m_logger;
    const std::string                                       m_planning_group = "kerby_arm_group";
    moveit::planning_interface::MoveGroupInterface          m_move_group;
    moveit::planning_interface::PlanningSceneInterface      m_planning_scene_interface;
    const moveit::core::JointModelGroup*                    m_joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan    m_plan;
};