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
    m_move_group = new moveit::planning_interface::MoveGroupInterface(shared_from_this(), m_planning_group);
    m_planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    m_joint_model_group = m_move_group->getCurrentState()->getJointModelGroup(m_planning_group);

    setScalingFactors(1, 1);

    initCommunication();
}

void Planner::initCommunication() {
    m_pose_target_sub = this->create_subscription<kerby_interfaces::msg::PoseGoal>("/HD/kinematics/pose_goal", 10, std::bind(&Planner::poseTargetCallback, this, _1));
    m_joint_target_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/kinematics/joint_goal", 10, std::bind(&Planner::jointTargetCallback, this, _1));
    m_add_object_sub = this->create_subscription<kerby_interfaces::msg::Object>("/HD/kinematics/add_object", 10, std::bind(&Planner::addObjectCallback, this, _1));
    m_mode_change_sub = this->create_subscription<std_msgs::msg::Int8>("/HD/fsm/mode_change", 10, std::bind(&Planner::modeChangeCallback, this, _1));
    m_man_inv_axis_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/HD/fsm/man_inv_axis_cmd", 10, std::bind(&Planner::manualInverseAxisCallback, this, _1));
    m_named_target_sub = this->create_subscription<std_msgs::msg::String>("/HD/kinematics/named_joint_target", 10, std::bind(&Planner::namedTargetCallback, this, _1));
    m_eef_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/HD/kinematics/eef_pose", 10);
    m_traj_feedback_pub = this->create_publisher<std_msgs::msg::Bool>("/HD/kinematics/traj_feedback", 10);
    m_position_mode_switch_pub = this->create_publisher<std_msgs::msg::Int8>("/HD/kinematics/position_mode_switch", 10);
}

Planner::TrajectoryStatus Planner::reachTargetPose(const geometry_msgs::msg::Pose &target)
{
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    updateCurrentPosition();
    m_move_group->setPoseTarget(target);
    return planAndExecute();
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

Planner::TrajectoryStatus Planner::computeCartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints, double velocity_scaling_factor=0.2) { 
    // TODO: make this function cleaner
    if (!canMove()) return Planner::TrajectoryStatus::CANNOT_ATTEMPT;

    updateCurrentPosition();
    Planner::TrajectoryStatus status = Planner::TrajectoryStatus::SUCCESS;

    // create cartesian path as in the tutorial
    //moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::RobotTrajectory trajectory_slow;

    //moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 10.0; // TODO: check how to put a real value here
    const double eef_step = 0.01;
    double fraction = m_move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_slow);

    // // add timing Note: you have to convert it to a RobotTrajectory Object (not message) and back
    // trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    // robot_trajectory::RobotTrajectory r_trajec(m_move_group->getRobotModel(), m_planning_group);
    // r_trajec.setRobotTrajectoryMsg(*m_move_group->getCurrentState(), trajectory_slow);
    // //iptp.computeTimeStamps(r_trajec, 1, 1);
    // //r_trajec.getRobotTrajectoryMsg(trajectory);
    // iptp.computeTimeStamps(r_trajec, 0.1, 0.1);
    // r_trajec.getRobotTrajectoryMsg(trajectory_slow);

    robot_trajectory::RobotTrajectory rt(m_move_group->getCurrentState()->getRobotModel(), m_planning_group);
    rt.setRobotTrajectoryMsg(*m_move_group->getCurrentState(), trajectory_slow);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ItSuccess = iptp.computeTimeStamps(rt, velocity_scaling_factor);
    RCLCPP_INFO(this->get_logger(), "Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_slow);
    //m_plan.trajectory_ = trajectory_slow;

    //double fraction = m_move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_slow);

    if (0 && fraction != 1.0)   // TODO
        status = Planner::TrajectoryStatus::PLANNING_ERROR;
    else if (!execute(trajectory_slow))
        status = Planner::TrajectoryStatus::EXECUTION_ERROR;

    sendTrajFeedback(status);
    return status;
}





// void setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string end_effector, const double speed)
// {
//     robot_model_loader::RobotModelLoader robot_model_loader(this, "robot_description");
//     robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//     kinematic_state->setToDefaultValues();
 
//     int num_waypoints = plan.trajectory_.joint_trajectory.points.size();                                //gets the number of waypoints in the trajectory
//     const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory
 
//     //set joint positions of zeroth waypoint
//     kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);
 
//     Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
//     Eigen::Affine3d next_end_effector_state;
//     double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
//     trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;
 
//     for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
//     {
//         curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
//         next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
         
//         //set joints for next waypoint
//         kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
//         //do forward kinematics to get cartesian positions of end effector for next waypoint
//         next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
//         //get euclidean distance between the two waypoints
//         euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
//                             pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
//                             pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
//         new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / speed);      //start by printing out all 3 of these!
//         old_timestamp = next_waypoint->time_from_start.toSec();
 
//         //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
//         if(new_timestamp > old_timestamp)
//             next_waypoint->time_from_start.fromSec(new_timestamp);
//         else
//         {
//             //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
//         }
         
//         //update current_end_effector_state for next iteration
//         current_end_effector_state = next_end_effector_state;
//     }
     
//     //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
//     for(int i = 0; i < num_waypoints; i++)
//     {
//         curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);            //set current, previous & next waypoints
//         if(i > 0)
//             prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i-1);
//         if(i < num_waypoints-1)
//             next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
 
//         if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
//             dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
//         else if(i < num_waypoints-1)
//         {
//             dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
//             dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
//         }
//         else
//             dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
//         for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
//         {
//             if(i == 0)                      //first point
//             {
//                 q1 = next_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }
//             else if(i < num_waypoints-1)    //middle points
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = next_waypoint->positions.at(j);
//             }
//             else                            //last point
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }
 
//             if(dt1 == 0.0 || dt2 == 0.0)
//                 v1 = v2 = a = 0.0;
//             else
//             {
//                 v1 = (q2 - q1)/dt1;
//                 v2 = (q3 - q2)/dt2;
//                 a = 2.0*(v2 - v1)/(dt1+dt2);
//             }
 
//             //actually set the velocity and acceleration
//             curr_waypoint->velocities.at(j) = (v1+v2)/2;
//             curr_waypoint->accelerations.at(j) = a;
//         }
//     }
// }






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
    step = pointImage(step, pose.orientation);
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
    //const double *positions = current_state.getVariablePositions();
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
    m_man_inv_axis[0] = 0.0;
    m_man_inv_axis[1] = 0.0;
    m_man_inv_axis[2] = 0.0;
}

void Planner::poseTargetCallback(const kerby_interfaces::msg::PoseGoal::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose goal");
    //setScalingFactors(msg->velocity_scaling_factor, msg->velocity_scaling_factor);

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
        std::thread executor(&Planner::advanceAlongAxis, this);
        executor.detach();
        exec_locked = false;
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
        m_mode_transition_ready = false;
        std::thread executor(&Planner::enforceCurrentState, this);
        executor.detach();
    }
    m_mode = new_mode;
}

void Planner::namedTargetCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received named target goal : " + msg->data);
    std::thread executor(&Planner::reachNamedTarget, this, msg->data);
    executor.detach();
}

void Planner::publishEEFPose()
{
    geometry_msgs::msg::Pose msg = m_move_group->getCurrentPose().pose;
    m_eef_pose_pub->publish(msg);
}

void Planner::sendTrajFeedback(Planner::TrajectoryStatus status) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std_msgs::msg::Bool msg;
    msg.data = (status == Planner::TrajectoryStatus::SUCCESS);
    m_traj_feedback_pub->publish(msg);
    m_is_executing_path = false;
}

bool Planner::canMove() {
    bool res = !m_is_executing_path;
    m_is_executing_path = true;
    return res;
}