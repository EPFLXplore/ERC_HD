#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "std_msgs/msg/int8.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using std::placeholders::_1;


class ServoPlanner : public rclcpp::Node {
public:
    enum CommandMode {
        IDLE = -1,
        MANUAL_INVERSE = 0,
        MANUAL_DIRECT = 1,
        SEMI_AUTONOMOUS = 2,
        AUTONOMOUS = 3
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

    void modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg);


    const std::string                                                       m_planning_group = "kerby_arm_group";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>         m_move_group;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>     m_planning_scene_interface;
    const moveit::core::JointModelGroup*                                    m_joint_model_group;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                    m_mode_change_sub;
    CommandMode                                                             m_mode = CommandMode::MANUAL_DIRECT;
};
