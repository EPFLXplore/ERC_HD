#include <hw_interface/hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <iostream>

namespace astra_ns
{
HWInterface::HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  fdbk_sub = nh.subscribe("/arm_control/joint_telemetry", 1, &HWInterface::fdbkCallback, this);
  cmd_pub = nh.advertise<sensor_msgs::JointState>("/arm_control/joint_cmd", 1);
  sim_cmd_pub = nh.advertise<motor_control::simJointState>("/arm_control/sim_joint_cmd", 1);
  ROS_INFO("HWInterface constructed");

  try
  {
    for (int i=0; i<num_joints_; i++)
    {
      previous_angle_command_[i] = joint_position_[i];
    }
    ROS_WARN("error not in constructor");
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtf constructor");
  }
  ROS_WARN("ye error def not in constructor");
}

void HWInterface::fdbkCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (int i=0; i<num_joints_; i++)
  {
    joint_position_[i] = msg->position[i];
    joint_velocity_[i] = msg->velocity[i];
  }
}

void HWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("MyHWInterface Ready.");
}

void HWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
  ros::spinOnce();
}

void HWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);


  sensor_msgs::JointState joint_cmd;
  static motor_control::simJointState sim_joint_cmd; // for simulation only

  bool new_cmd = false;
  //ROS_WARN("entering HI write");
  try
  {
    //ROS_WARN("checkpoint1");
    for (int i=0; i<num_joints_; i++)
  {
    //ROS_WARN("checkpoint2");
    joint_cmd.position.push_back(joint_position_command_[i]);
    sim_joint_cmd.position[i] = joint_position_command_[i];
    //ROS_WARN("checkpoint2.5");
    if (previous_angle_command_[i] != joint_cmd.position[i])
    {
      //ROS_WARN("checkpoint3");
      new_cmd = true;
      joint_cmd.velocity.push_back((joint_cmd.position[i]-previous_angle_command_[i])/elapsed_time.toSec());
      sim_joint_cmd.velocity[i] = (sim_joint_cmd.position[i]-previous_angle_command_[i])/elapsed_time.toSec();
      //ROS_WARN("checkpoint4");
      //ROS_WARN("checkpoint5");
      previous_angle_command_[i] = joint_cmd.position[i];
      //ROS_WARN("checkpoint6");
    }
  }
  //ROS_WARN("checkpoint7");
  if (new_cmd)
  {
    //ROS_WARN("new command");
    cmd_pub.publish(joint_cmd);
    sim_cmd_pub.publish(sim_joint_cmd);
  }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtfffffff");
  }


}

void HWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}



}  // namespace atra_ns
