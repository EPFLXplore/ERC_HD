// ROS
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr manual_direct_cmd_sub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr manual_inverse_cmd_sub_;

enum CommandMode {
        MANUAL_INVERSE = 0,
        MANUAL_DIRECT = 1,
        SEMI_AUTONOMOUS = 2,
        AUTONOMOUS = 3
    };

CommandMode mode_ = MANUAL_DIRECT;

std::vector<double> joint_velocities_ = {0, 0, 0, 0, 0, 0, 0, 0};
std::vector<double> inverse_cmds_ = {0, 0, 0, 0, 0, 0};

const std::vector<std::string> joints_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

void directCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    mode_ = MANUAL_DIRECT;
    for (size_t i; i < msg->data.size(); i++) {
        joint_velocities_[i] = msg->data[i];
    }
}

void inverseCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    mode_ = MANUAL_INVERSE;
    for (size_t i; i < msg->data.size(); i++) {
        inverse_cmds_[i] = msg->data[i];
    }
}

void publishCommands()
{
    if (mode_ == MANUAL_DIRECT) {
        auto msg = std::make_unique<control_msgs::msg::JointJog>();
        msg->header.stamp = node_->now();
        for (size_t i = 0; i < 6; i++) {
            msg->joint_names.push_back(joints_[i]);
            msg->velocities.push_back(joint_velocities_[i]);
        }
        joint_cmd_pub_->publish(std::move(msg));
    }

    else if (mode_ == MANUAL_INVERSE) {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = "base_link";
        msg->twist.linear.x = inverse_cmds_[1];
        msg->twist.linear.y = inverse_cmds_[2];
        msg->twist.linear.z = inverse_cmds_[3];
        twist_cmd_pub_->publish(std::move(msg));
    }

}

// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  //rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
  // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
  // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10);
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10);
  manual_direct_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/CS/HD_gamepad", 10, std::bind(&directCallback, _1));
  manual_inverse_cmd_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/ROVER/HD_man_inv_axis", 10, std::bind(&inverseCallback, _1));

  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";

  // Make a box and put it in the way
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.6;

  // Add the box as a collision object
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);

  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  servo->start();

  // Sending Commands
  // ^^^^^^^^^^^^^^^^
  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  // CALL_SUB_TUTORIAL publishCommands

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}
