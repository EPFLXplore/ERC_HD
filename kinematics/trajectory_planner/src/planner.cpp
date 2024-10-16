#include "trajectory_planner/planner.h"
#include "trajectory_planner/quaternion_arithmetic.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


using namespace std::chrono_literals;

Planner::Planner(rclcpp::NodeOptions node_options) : Node("kinematics_trajectory_planner", node_options)
{}

void Planner::config()
{
    m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), m_planning_group);
    m_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);

    m_move_group->setPlannerId("PRMstar");

    setScalingFactors(1, 1);

    // addDronePlatform();
    // addLidar();
    std::vector<double> shape = {0.05, 0.1, 0.3};
    const geometry_msgs::msg::Pose pose;
    std::string name = "attach_test";
    // addGripperAttachedBoxToWorld(shape, pose, name);

    initCommunication();
}

void Planner::initCommunication() {
    m_pose_target_sub = this->create_subscription<custom_msg::msg::PoseGoal>("/HD/kinematics/pose_goal", 10, std::bind(&Planner::poseTargetCallback, this, _1));
    m_joint_target_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_goal", 10, std::bind(&Planner::jointTargetCallback, this, _1));
    m_joint_target2_sub = this->create_subscription<custom_msg::msg::JointSpaceCmd>("/HD/kinematics/joint_goal2", 10, std::bind(&Planner::jointTarget2Callback, this, _1));
    m_add_object_sub = this->create_subscription<custom_msg::msg::Object>("/HD/kinematics/add_object", 10, std::bind(&Planner::addObjectCallback, this, _1));
    m_attach_object_sub = this->create_subscription<custom_msg::msg::Object>("/HD/kinematics/attach_object", 10, std::bind(&Planner::attachGripperObjectCallback, this, _1));
    m_add_object2_sub = this->create_subscription<moveit_msgs::msg::CollisionObject>("/HD/kinematics/add_object2", 10, std::bind(&Planner::addObjectToWorld, this, _1));
    m_mode_change_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/fsm/mode_change", 10, std::bind(&Planner::modeChangeCallback, this, _1));
    m_man_inv_axis_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/fsm/man_inv_axis_cmd", 10, std::bind(&Planner::manualInverseAxisCallback, this, _1));
    m_named_target_sub = this->create_subscription<std_msgs::msg::String>("/HD/kinematics/named_joint_target", 10, std::bind(&Planner::namedTargetCallback, this, _1));
    m_cs_maintenance_sub = this->create_subscription<std_msgs::msg::Int8>("/ROVER/Maintenance", 10, std::bind(&Planner::CSMaintenanceCallback, this, _1));
    m_abort_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/fsm/abort", 10, std::bind(&Planner::abort, this, _1));
    m_man_inv_frame_sub = this->create_subscription<std_msgs::msg::String>("ROVER/HD_inverse_frame", 10, std::bind(&Planner::manualInverseFrameCallback, this, _1));
    m_eef_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/HD/kinematics/eef_pose", 10);
    m_traj_feedback_pub = this->create_publisher<std_msgs::msg::Bool>("/HD/kinematics/traj_feedback", 10);
    m_position_mode_switch_pub = this->create_publisher<std_msgs::msg::Int8>("/HD/kinematics/position_mode_switch", 10);
    m_sanity_feedback_pub = this->create_publisher<std_msgs::msg::Int8>("/HD/kinematics/planner_sanity_feedback", 10);
}

Planner::TrajectoryStatus Planner::reachTargetPose(const geometry_msgs::msg::Pose &target, double velocity_scaling_factor)
{
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    setScalingFactors(velocity_scaling_factor, velocity_scaling_factor);
    updateCurrentPosition();
    
    // moveit_msgs::msg::PositionConstraint box_constraint;
    // box_constraint.header.frame_id = m_move_group->getPoseReferenceFrame();
    // box_constraint.link_name = m_move_group->getEndEffectorLink();
    // shape_msgs::msg::SolidPrimitive box;
    // box.type = shape_msgs::msg::SolidPrimitive::BOX;
    // box.dimensions = { 1.0, 1.0, 1.0 };
    // box_constraint.constraint_region.primitives.emplace_back(box);

    // geometry_msgs::msg::Pose box_pose;
    // geometry_msgs::msg::Pose current_pose = m_move_group->getCurrentPose().pose;
    // box_pose.position.x = current_pose.position.x;
    // box_pose.position.y = current_pose.position.y;
    // box_pose.position.z = current_pose.position.z;
    // box_pose.orientation.w = 1.0;
    // box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    // box_constraint.weight = 1.0;

    // moveit_msgs::msg::Constraints box_constraints;
    // box_constraints.position_constraints.emplace_back(box_constraint);
    // m_move_group->setPathConstraints(box_constraints);
    
    m_move_group->setPoseTarget(target);
    // m_move_group->setPlanningTime(10.0);
    Planner::TrajectoryStatus status = planAndExecute();
    setScalingFactors(1, 1);
    return status;
}

Planner::TrajectoryStatus Planner::reachTargetJointValues(const std::vector<double> &target)
{
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    updateCurrentPosition();
    m_move_group->setJointValueTarget(target);
    return planAndExecute();
}

Planner::TrajectoryStatus Planner::reachNamedTarget(const std::string &target)
{
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    updateCurrentPosition();
    m_move_group->setNamedTarget(target);
    return planAndExecute();
}

Planner::TrajectoryStatus Planner::computeCartesianPath(std::vector<geometry_msgs::msg::Pose> &waypoints) {
    computeCartesianPath(waypoints, 1.0);
}

Planner::TrajectoryStatus Planner::computeCartesianPath(std::vector<geometry_msgs::msg::Pose> &waypoints, double velocity_scaling_factor) { 
    // TODO: make this function cleaner
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    updateCurrentPosition();
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;

    // compute trajectory
    moveit_msgs::msg::RobotTrajectory trajectory_slow;
    const double jump_threshold = 10.0; // TODO: check how to put a real value here
    const double eef_step = 0.01;
    double fraction = m_move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_slow);

    // retime it to potentially slow it down
    robot_trajectory::RobotTrajectory rt(m_move_group->getCurrentState()->getRobotModel(), m_planning_group);
    rt.setRobotTrajectoryMsg(*m_move_group->getCurrentState(), trajectory_slow);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ItSuccess = iptp.computeTimeStamps(rt, velocity_scaling_factor);
    RCLCPP_INFO(this->get_logger(), "Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_slow);

    if (0 && fraction != 1.0)   // TODO
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute(trajectory_slow))
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    sendTrajFeedback(status);
    return status;
}

Planner::TrajectoryStatus Planner::reachTargetPoseCartesian(const geometry_msgs::msg::Pose &target, double velocity_scaling_factor) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    return computeCartesianPath(waypoints, velocity_scaling_factor);
}

Planner::TrajectoryStatus Planner::advanceAlongAxis() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double step_size = 0.03;     // [m]
    double limit = 0.03 + m_man_inv_velocity_scaling * 0.10;      // 2.0   // [m]
    int step_count = std::ceil(limit/step_size);
    geometry_msgs::msg::Point step;
    step.x = m_man_inv_axis[0]*step_size;
    step.y = m_man_inv_axis[1]*step_size;
    step.z = m_man_inv_axis[2]*step_size;
    geometry_msgs::msg::Pose pose = m_move_group->getCurrentPose().pose;
    if (m_man_inv_gripper_frame) {
        step = pointImage(step, pose.orientation);
    }
    for (int i = 1; i <= step_count; i++) {
        pose.position.x += step.x;
        pose.position.y += step.y;
        pose.position.z += step.z;
        geometry_msgs::msg::Pose waypoint = pose;
        waypoints.push_back(waypoint);
    }
    Planner::TrajectoryStatus status = computeCartesianPath(waypoints, m_man_inv_velocity_scaling);
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

Planner::TrajectoryStatus Planner::planAndExecute() {
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;
    if (!plan()) {
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    }
    else if (!execute()) {
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;
    }


    sendTrajFeedback(status);
    return status;
}

void Planner::enforceCurrentState()     // TODO: maybe add in this method the canMove and sendTrajFeedback (or something similar without sending a message)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    updateCurrentPosition();
    moveit::core::RobotState current_state(*m_move_group->getCurrentState());
    const double *positions = current_state.getVariablePositions();
    std::vector<double> target;
    static size_t MOTOR_COUNT = 6;
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
        m_mode_transition_ready = true;
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

void Planner::addDronePlatform() {
    std::vector<double> shape = {0.5, 0.5, 0.02};
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.13;
    pose.position.y = 0.02;
    pose.position.z = 0.74;
    std::string name = "drone_platform";
    addBoxToWorld(shape, pose, name);
}

void Planner::addLidar() {
    std::vector<double> shape = {0.12, 0.12, 1};
    geometry_msgs::msg::Pose pose;
    pose.position.x = -0.4;
    pose.position.y = -0.39;
    pose.position.z = 0.58;
    std::string name = "lidar";
    addBoxToWorld(shape, pose, name);
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

    RCLCPP_INFO(this->get_logger(), "Add object '%s' into the world", name);
    m_planning_scene_interface->addCollisionObjects(collision_objects);
}

void Planner::addGripperAttachedBoxToWorld(const std::vector<double> &shape, const geometry_msgs::msg::Pose &pose, std::string &name) {
    moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link6";
    collision_object.id = name;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

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

    attached_collision_object.object = collision_object;
    attached_collision_object.link_name = "link6";
    attached_collision_object.touch_links = std::vector<std::string>{"link5", "link6", "Arm_A_v6_2", "Arm_A_v6_1", "Finger_v1_2", "Finger_v1_1"};

    std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_collision_objects;
    attached_collision_objects.push_back(attached_collision_object);

    RCLCPP_INFO(this->get_logger(), "Add object '%s' into the world", name);
    m_planning_scene_interface->addCollisionObjects(collision_objects);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    m_planning_scene_interface->applyAttachedCollisionObject(attached_collision_object);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Planner::removeGripperAttachedBoxToWorld(const std::vector<double> &shape, const geometry_msgs::msg::Pose &pose, std::string &name) {
    moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link6";
    collision_object.id = name;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Define the box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = shape[0];
    primitive.dimensions[primitive.BOX_Y] = shape[1];
    primitive.dimensions[primitive.BOX_Z] = shape[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.REMOVE;

    attached_collision_object.object = collision_object;
    attached_collision_object.link_name = "link6";
    attached_collision_object.touch_links = std::vector<std::string>{"link6", "Arm_A_v6_2", "Arm_A_v6_1", "Finger_v1_2", "Finger_v1_1"};

    std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_collision_objects;
    attached_collision_objects.push_back(attached_collision_object);

    RCLCPP_INFO(this->get_logger(), "Add object '%s' into the world", name);
    m_planning_scene_interface->applyAttachedCollisionObject(attached_collision_object);
    m_planning_scene_interface->addCollisionObjects(collision_objects);
}

void Planner::addObjectToWorld(const moveit_msgs::msg::CollisionObject::SharedPtr object) {
    // // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    // // collision_objects.push_back(object);
    // object->header.frame_id = m_move_group->getPlanningFrame();

    // RCLCPP_INFO(this->get_logger(), "Add object '%s' into the world", object->id);
    // m_planning_scene_interface->addCollisionObjects(object);
}

void Planner::removeFromWorld(std::string &name) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = m_move_group->getPlanningFrame();

    collision_object.id = name;
    collision_object.operation = collision_object.REMOVE;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(this->get_logger(), "Remove object '%s' from world", name);
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
        publishSanityFeedback();
        switch(m_mode) {
            case Planner::CommandMode::IDLE:
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
    // moveit::core::RobotState current_state(*m_move_group->getCurrentState());
    // //const double *positions = current_state.getVariablePositions();
    // m_move_group->setStartState(current_state);
    // current_state.update(true);
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
    m_man_inv_axis[0] = 0.0;
    m_man_inv_axis[1] = 0.0;
    m_man_inv_axis[2] = 0.0;
}

void Planner::poseTargetCallback(const custom_msg::msg::PoseGoal::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose goal");
    //setScalingFactors(msg->velocity_scaling_factor, msg->velocity_scaling_factor);

    if (msg->cartesian) {
        std::thread executor(&Planner::reachTargetPoseCartesian, this, msg->goal, msg->velocity_scaling_factor);
        executor.detach();
    }
    else {
        std::thread executor(&Planner::reachTargetPose, this, msg->goal, msg->velocity_scaling_factor);
        executor.detach();
    }
}

void Planner::jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received joint goal");
    std::thread executor(&Planner::reachTargetJointValues, this, msg->data);
    executor.detach();
}

void Planner::jointTarget2Callback(const custom_msg::msg::JointSpaceCmd::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received joint goal ***");
    std::thread executor(&Planner::jointTargetIntermediary, this, msg);
    executor.detach();
}

void Planner::jointTargetIntermediary(const custom_msg::msg::JointSpaceCmd::SharedPtr msg) {
    moveit::core::RobotStatePtr current_state = m_move_group->getCurrentState();
    std::vector<double> joint_group_positions = {0, 0, 0, 0, 0, 0, 0};
    current_state->copyJointGroupPositions(m_joint_model_group, joint_group_positions);
    for (size_t i=0; i < 7; i++) {
        if (msg->states.data[i] != msg->CURRENT_STATE) {
            if (msg->mode == msg->ABSOLUTE) joint_group_positions[i] = msg->states.data[i];
            else if (msg->mode == msg->RELATIVE) joint_group_positions[i] += msg->states.data[i];
        }
    }
    reachTargetJointValues(joint_group_positions);
}

bool equal(const std::vector<double> &v1, const std::vector<double> &v2) {
    // TODO: establish wether to also compare speeds or not
    return v1[0] == v2[0] && v1[1]  == v2[1] && v1[2] == v2[2];
}

bool isZero(const std::vector<double> &v) {
    return (v[0] == 0 && v[1] == 0 && v[2] == 0) || v[3] == 0;      // v[3] is the speed scaling
}

void Planner::manualInverseAxisCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // last element of the message is the velocity scaling, the first three represent the axis
    if (!m_mode_transition_ready) return;

    if (isZero(msg->data)) {
        stop();
        return;
    }

    static bool exec_locked = false;
    if (!exec_locked && (!m_executing_man_inv_cmd || !equal(m_man_inv_axis, msg->data))) {
        exec_locked = true;
        if (!equal(m_man_inv_axis, msg->data)) {
            stop();
            static const double security_sleep = 1000;      // In order to make sure not to send commands to MoveIt while old ones are still executing, won't move in new direction for 1 second after last command
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now-m_last_man_inv_cmd_time).count() < security_sleep) {
                exec_locked = false;
                return;
            }
        }
        m_executing_man_inv_cmd = true;
        m_man_inv_axis = msg->data;
        m_man_inv_velocity_scaling = msg->data[3];
        std::thread executor(&Planner::advanceAlongAxis, this);
        executor.detach();
        exec_locked = false;
    }
    
    m_last_man_inv_cmd_time = std::chrono::steady_clock::now();


    // if (!m_executing_man_inv_cmd) {
    //     if (!equal(m_man_inv_axis, msg->data)) {
    //         stop();
    //     }
    //     m_executing_man_inv_cmd = true;
    //     m_man_inv_axis = msg->data;
    //     std::thread executor(&Planner::advanceAlongAxis, this);
    //     executor.detach();
    // }
}

void Planner::addObjectCallback(const custom_msg::msg::Object::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new object");
    if (msg->operation == msg->REMOVE) {
        removeFromWorld(msg->name);
        return;
    }
    if (msg->type == msg->BOX) addBoxToWorld(msg->shape.data, msg->pose, msg->name);
}

void Planner::attachGripperObjectCallback(const custom_msg::msg::Object::SharedPtr msg)
{
    if (msg->type != msg->BOX) return;  // not implemented
    RCLCPP_INFO(this->get_logger(), "Received new object to attach on gripper");
    if (msg->operation == msg->REMOVE) {
        removeGripperAttachedBoxToWorld(msg->shape.data, msg->pose, msg->name);
    }
    else {
        addGripperAttachedBoxToWorld(msg->shape.data, msg->pose, msg->name);
    }
}

void Planner::modeChangeCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    Planner::CommandMode new_mode = static_cast<Planner::CommandMode>(msg->data);
    if (new_mode != Planner::CommandMode::MANUAL_DIRECT && new_mode != Planner::CommandMode::IDLE) {
        m_mode_transition_ready = false;
        std::thread executor(&Planner::enforceCurrentState, this);
        executor.detach();
    }
    m_mode = new_mode;
}

void Planner::namedTargetCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received named target goal : %s", msg->data);
    std::thread executor(&Planner::reachNamedTarget, this, msg->data);
    executor.detach();
}

void Planner::CSMaintenanceCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    static const int LAUNCH = 1;
    static const int ABORT = 2;
    static const int WAIT = 3;
    static const int RESUME = 4;
    static const int CANCEL = 5;
    switch(msg->data) {
        case ABORT:
        case CANCEL:
            stop();
            if (m_is_executing_path) sendTrajFeedback(Planner::TrajectoryStatus::EXECUTION_ERROR);
            break;
    }
}

void Planner::abort(const std_msgs::msg::Int8::SharedPtr msg) {
    stop();
    if (m_is_executing_path) sendTrajFeedback(Planner::TrajectoryStatus::ABORTED);
}

void Planner::manualInverseFrameCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "gripper") m_man_inv_gripper_frame = true;
    if (msg->data == "rover") m_man_inv_gripper_frame = false;
}

void Planner::publishEEFPose()
{
    geometry_msgs::msg::Pose msg = m_move_group->getCurrentPose().pose;
    m_eef_pose_pub->publish(msg);
}

void Planner::publishSanityFeedback() {
    std_msgs::msg::Int8 msg;
    m_sanity_feedback_pub->publish(msg);
}

void Planner::sendTrajFeedback(Planner::TrajectoryStatus status) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    m_is_executing_path = false;
}

bool Planner::canMove() {
    m_move_group->clearPathConstraints();
    bool res = !m_is_executing_path;
    m_is_executing_path = true;
    return res;
}