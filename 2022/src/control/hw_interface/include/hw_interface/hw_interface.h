#ifndef ASTRA_HW_INTERFACE_H
#define ASTRA_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <sensor_msgs/JointState.h>
#include <motor_control/simJointState.h>  // for simulation only


namespace astra_ns
{
/** \brief Hardware interface for a robot */
class HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:

  ros::Subscriber fdbk_sub;
  void fdbkCallback(const sensor_msgs::JointState::ConstPtr &msg);

  ros::Publisher cmd_pub;

  ros::Publisher sim_cmd_pub; // for simulation only

  double previous_angle_command_[6];

};  // class

}  // namespace ros_control_boilerplate

#endif
