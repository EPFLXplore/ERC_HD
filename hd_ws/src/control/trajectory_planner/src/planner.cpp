#include "trajectory_planner/planner.h"
#include "trajectory_planner/quaternion_arithmetic.h"


using namespace std::chrono_literals;

Planner::Planner(rclcpp::NodeOptions node_options) : Node("kinematics_trajectory_planner", node_options)
{
    m_pose_target_sub = this->create_subscription<kerby_interfaces::msg::PoseGoal>("/HD/kinematics/pose_goal", 10, std::bind(&Planner::poseTargetCallback, this, _1));
    m_joint_target_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_goal", 10, std::bind(&Planner::jointTargetCallback, this, _1));
    m_add_object_sub = this->create_subscription<kerby_interfaces::msg::Object>("/HD/kinematics/add_object", 10, std::bind(&Planner::addObjectCallback, this, _1));
    m_mode_change_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/fsm/mode_change", 10, std::bind(&Planner::modeChangeCallback, this, _1));
    m_man_inv_axis_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/fsm/man_inv_axis_cmd", 10, std::bind(&Planner::manualInverseAxisCallback, this, _1));

    m_eef_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/HD/kinematics/eef_pose", 10);
    m_traj_feedback_pub = this->create_publisher<std_msgs::msg::Bool>("/HD/kinematics/traj_feedback", 10);
    m_position_mode_switch_pub = this->create_publisher<std_msgs::msg::Int8>("/HD/kinematics/position_mode_switch", 10);
}

void Planner::config()
{
    m_move_group = new moveit::planning_interface::MoveGroupInterface(shared_from_this(), m_planning_group);
    m_planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);

    setScalingFactors(1, 1);
}

Planner::TrajectoryStatus Planner::reachTargetPose(const geometry_msgs::msg::Pose &target)
{
    updateCurrentPosition();
    m_move_group->setPoseTarget(target);
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;
    if (!plan())
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute())
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

Planner::TrajectoryStatus Planner::reachTargetJointValues(const std::vector<double> &target)
{
    updateCurrentPosition();
    m_move_group->setJointValueTarget(target);
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;
    if (!plan())
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute())
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

Planner::TrajectoryStatus Planner::computeCartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints) { 
    // TODO: make this function cleaner
    updateCurrentPosition();
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // TODO: check how to put a real value here
    const double eef_step = 0.01;
    double fraction = m_move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (0 && fraction != 1.0)
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute(trajectory))
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    return status;
}

Planner::TrajectoryStatus Planner::reachTargetPoseCartesian(const geometry_msgs::msg::Pose &target) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    return computeCartesianPath(waypoints);
}

Planner::TrajectoryStatus Planner::advanceAlongAxis() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double step_size = 0.1;     // [m]
    double limit = 2.0;         // [m]
    int step_count = std::ceil(limit/step_size);
    geometry_msgs::msg::Point step;
    step.x = m_man_inv_axis[0]*step_size;
    step.y = m_man_inv_axis[1]*step_size;
    step.z = m_man_inv_axis[2]*step_size;
    geometry_msgs::msg::Pose pose = m_move_group->getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(), "Step before : %g, %g, %g", step.x, step.y, step.z);
    //RCLCPP_INFO(this->get_logger(), "Quat : w: %g, x: %g, y: %g, z: %g", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    step = pointImage(step, pose.orientation);
    RCLCPP_INFO(this->get_logger(), "Step after : %g, %g, %g", step.x, step.y, step.z);
    for (int i = 1; i <= step_count; i++) {
        pose.position.x += step.x;
        pose.position.y += step.y;
        pose.position.z += step.z;
        geometry_msgs::msg::Pose waypoint = pose;
        waypoints.push_back(waypoint);
    }
    Planner::TrajectoryStatus status = computeCartesianPath(waypoints);
    m_executing_man_inv_cmd = false;
    return status;
}

bool Planner::plan()
{
    return (m_move_group->plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::execute()
{
    return (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::executeSilent()
{
    return (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool Planner::execute(moveit_msgs::msg::RobotTrajectory &trajectory)
{
    return (m_move_group->execute(trajectory) != moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void Planner::enforceCurrentState() 
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateCurrentPosition();
    moveit::core::RobotState current_state(*m_move_group->getCurrentState());
    const double *positions = current_state.getVariablePositions();
    std::vector<double> target;
    static size_t MOTOR_COUNT = 7;
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        target.push_back(positions[i]);
    }

    m_move_group->setJointValueTarget(target);

    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;

    if (!plan())
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!executeSilent())
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    std_msgs::msg::Int8 msg;
    if (status == Planner::TrajectoryStatus::SUCCESS) {
        msg.data = 1;
        RCLCPP_INFO(this->get_logger(), "Successful switch to position mode with MoveIt.");
    }
    else {
        msg.data = 0;
        RCLCPP_INFO(this->get_logger(), "Failed to switch to position mode with MoveIt, no commands will be executed.");
    }
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_position_mode_switch_pub->publish(msg);
}

void Planner::setScalingFactors(double vel, double accel)
{
    m_move_group->setMaxVelocityScalingFactor(vel);
    m_move_group->setMaxAccelerationScalingFactor(accel);
}

void Planner::addBoxToWorld(const std::vector<double> &shape, const geometry_msgs::msg::Pose &pose, std::string &name)
{
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

void Planner::spin()
{
    rclcpp::spin(shared_from_this());
}

void Planner::loop()
{
    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        publishEEFPose();
        switch(m_mode) {
            case Planner::CommandMode::MANUAL_DIRECT:
                updateCurrentPosition();
                break;
            case Planner::CommandMode::MANUAL_INVERSE:
                if (manualInverseCommandOld() && m_executing_man_inv_cmd) {
                    stop();
                }
                break;
        }
        rate.sleep();
    }
}

void Planner::updateCurrentPosition()
{
    moveit::core::RobotState current_state(*m_move_group->getCurrentState());
    const double *positions = current_state.getVariablePositions();
    m_move_group->setStartState(current_state);
    current_state.update(true);
}

bool Planner::manualInverseCommandOld() {
    static const double command_expiration = 200;
    auto now = std::chrono::steady_clock::now();
    return (std::chrono::duration_cast<std::chrono::milliseconds>(now-m_last_man_inv_cmd_time).count() > command_expiration);
}

void Planner::stop() {
    m_move_group->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    m_executing_man_inv_cmd = false;
}

void Planner::poseTargetCallback(const kerby_interfaces::msg::PoseGoal::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose goal");
    if (msg->cartesian) {
        std::thread executor(&Planner::reachTargetPoseCartesian, this, msg->goal);
        executor.detach();
    }
    else {
        std::thread executor(&Planner::reachTargetPose, this, msg->goal);
        executor.detach();
    }
}

void Planner::jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received joint goal");
    std::thread executor(&Planner::reachTargetJointValues, this, msg->data);
    executor.detach();
}

bool equal(const std::vector<double> &v1, const std::vector<double> &v2) {
    return v1[0] == v2[0] && v1[1]  == v2[1] && v1[2] == v2[2];
}

bool isZero(const std::vector<double> &v) {
    return v[0] == 0 && v[1] == 0 && v[2] == 0;
}

void Planner::manualInverseAxisCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (isZero(msg->data)) {
        stop();
        return;
    }

    if (!m_executing_man_inv_cmd || !equal(m_man_inv_axis, msg->data)) {
        if (m_executing_man_inv_cmd) stop();
        m_man_inv_axis = msg->data;
        std::thread executor(&Planner::advanceAlongAxis, this);
        executor.detach();
        m_executing_man_inv_cmd = true;
    }
    
    m_last_man_inv_cmd_time = std::chrono::steady_clock::now();
}

void Planner::addObjectCallback(const kerby_interfaces::msg::Object::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new object");
    if (msg->type == "box")
        addBoxToWorld(msg->shape.data, msg->pose, msg->name);
}

void Planner::modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    Planner::CommandMode new_mode = static_cast<Planner::CommandMode>(msg->data);
    if (new_mode != Planner::CommandMode::MANUAL_DIRECT) {
        std::thread executor(&Planner::enforceCurrentState, this);
        executor.detach();
    }
    m_mode = new_mode;
}

void Planner::publishEEFPose()
{
    geometry_msgs::msg::Pose msg = m_move_group->getCurrentPose().pose;
    m_eef_pose_pub->publish(msg);
}
