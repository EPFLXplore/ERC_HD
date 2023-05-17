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


using std::placeholders::_1;


class Planner : public rclcpp::Node {
public:
    enum TrajectoryStatus {
        SUCCESS,
        PLANNING_ERROR,
        EXECUTION_ERROR
    };

    Planner(rclcpp::NodeOptions node_options);

    void config();

    ~Planner() {    // TODO: make the pointer shared so no need for destructor
        delete m_move_group;
        delete m_planning_scene_interface;
        }

    TrajectoryStatus reachTargetPose(const geometry_msgs::msg::Pose &target);

    TrajectoryStatus reachTargetJointValues(const std::vector<double> &target);

    TrajectoryStatus computeCartesianPath(const geometry_msgs::msg::Pose &target);

    bool plan();

    bool execute();

    void setScalingFactors(double vel, double accel);

    void addBoxToWorld(const std::vector<double> &dim, const geometry_msgs::msg::Pose &pose, std::string &name);

    void spin();

    void loop();

private:
    void poseTargetCallback(const kerby_interfaces::msg::PoseGoal::SharedPtr msg);

    void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void addObjectCallback(const kerby_interfaces::msg::Object::SharedPtr msg);

    void publishEEFPose();

    // void poseTargetSrv(const std::shared_ptr<kerby_interfaces::srv::PoseGoal::Request>          request, 
    //                          std::shared_ptr<kerby_interfaces::srv::PoseGoal::Response>         response);

    // void jointTargetSrv(const std::shared_ptr<kerby_interfaces::srv::JointGoal::Request>        request, 
    //                           std::shared_ptr<kerby_interfaces::srv::JointGoal::Response>       response);


    const std::string                                                   m_planning_group = "kerby_arm_group";
    moveit::planning_interface::MoveGroupInterface*                     m_move_group;
    moveit::planning_interface::PlanningSceneInterface*                 m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                m_joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan                m_plan;
    rclcpp::Subscription<kerby_interfaces::msg::PoseGoal>::SharedPtr    m_pose_target_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   m_joint_target_sub;
    rclcpp::Subscription<kerby_interfaces::msg::Object>::SharedPtr      m_add_object_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr              m_eef_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   m_traj_feedback_pub;
};
