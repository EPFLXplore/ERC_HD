#include "trajectory_planner/planner.h"


using namespace std::chrono_literals;


Planner::Planner(rclcpp::NodeOptions node_options) : 
    Node("kinematics_trajectory_planner", node_options)
{
    m_pose_target_sub = this->create_subscription<kerby_interfaces::msg::PoseGoal>("/HD/kinematics/pose_goal", 10, std::bind(&Planner::poseTargetCallback, this, _1));
    m_joint_target_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_goal", 10, std::bind(&Planner::jointTargetCallback, this, _1));
    m_add_object_sub = this->create_subscription<kerby_interfaces::msg::Object>("/HD/kinematics/add_object", 10, std::bind(&Planner::addObjectCallback, this, _1));

    m_eef_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/HD/kinematics/eef_pose", 10);
    m_traj_feedback_pub = this->create_publisher<std_msgs::msg::Bool>("/HD/kinematics/traj_feedback", 10);
}

void Planner::config() {
    m_move_group = new moveit::planning_interface::MoveGroupInterface(shared_from_this(), m_planning_group);
    m_planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);

    setScalingFactors(1, 1);
}

Planner::TrajectoryStatus Planner::reachTargetPose(const geometry_msgs::msg::Pose &target) {
    m_move_group->setPoseTarget(target);
    Planner::TrajectoryStatus status;
    if (!plan()) status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute()) status = Planner::TrajectoryStatus::EXECUTION_ERROR;
    status = Planner::TrajectoryStatus::SUCCESS;
    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

Planner::TrajectoryStatus Planner::reachTargetJointValues(const std::vector<double> &target) {
    m_move_group->setJointValueTarget(target);
    Planner::TrajectoryStatus status;
    if (!plan()) status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute()) status = Planner::TrajectoryStatus::EXECUTION_ERROR;
    status = Planner::TrajectoryStatus::SUCCESS;
    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

Planner::TrajectoryStatus Planner::computeCartesianPath(const geometry_msgs::msg::Pose &target) {   // TODO: make this function cleaner
    Planner::TrajectoryStatus status;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // TODO: check how to put a real value here
    const double eef_step = 0.01;
    double fraction = m_move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction != 1.0) status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (m_move_group->execute(trajectory) != moveit::planning_interface::MoveItErrorCode::SUCCESS) 
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;
    status = Planner::TrajectoryStatus::SUCCESS;

    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

bool Planner::plan() {
    return (m_move_group->plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::execute() {
    return (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void Planner::setScalingFactors(double vel, double accel) {
    m_move_group->setMaxVelocityScalingFactor(vel);
    m_move_group->setMaxAccelerationScalingFactor(accel);
}

void Planner::addBoxToWorld(const std::vector<double> &shape, const geometry_msgs::msg::Pose &pose, std::string &name) {
    // TODO: make this function better

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = m_move_group->getPlanningFrame();

    collision_object.id = name;

    // Define the box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = shape[0];
    primitive.dimensions[primitive.BOX_Y] = shape[1];
    primitive.dimensions[primitive.BOX_Z] = shape[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(this->get_logger(), "Add an object into the world");
    m_planning_scene_interface->addCollisionObjects(collision_objects);
}

void Planner::loopBody() {
    publishEEFPose();
}

void Planner::spin1() {
    rclcpp::spin(shared_from_this());
}

void Planner::spin2() {
    while (rclcpp::ok()) {
        loopBody();
        // TODO: add some sleep here
        //rclcpp::spin_some(shared_from_this());
    }
}

void Planner::poseTargetCallback(const kerby_interfaces::msg::PoseGoal::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received pose goal");
    //m_pose_target = *msg;
    if (msg->cartesian) {
        std::thread executor(&Planner::computeCartesianPath, this, msg->goal);
        executor.detach();
    }
    else {
        std::thread executor(&Planner::reachTargetPose, this, msg->goal);
        executor.detach();
    }
}

void Planner::jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received joint goal");
    //m_joints_target = msg->data;
    std::thread executor(&Planner::reachTargetJointValues, this, msg->data);
    executor.detach();
}

void Planner::addObjectCallback(const kerby_interfaces::msg::Object::SharedPtr msg) {
    if (msg->type == "box") addBoxToWorld(msg->shape.data, msg->pose, msg->name);
}

void Planner::publishEEFPose() {
    geometry_msgs::msg::Pose msg = m_move_group->getCurrentPose().pose;
    m_eef_pose_pub->publish(msg);
}
