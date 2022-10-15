#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <hw_interface/hw_interface.h>

int main(int argc, char** argv)
{
  ROS_WARN("entering hw main");
  try
  {
    ros::init(argc, argv, "astra_hw_interface");
  ros::NodeHandle nh;
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();
  // Create the hardware interface specific to your robot
  boost::shared_ptr<astra_ns::HWInterface> astra_hw_interface(
      new astra_ns::HWInterface(nh));
  astra_hw_interface->init();
  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, astra_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved
  ROS_WARN("error not in hw main");
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtf hw main");
  }
  ROS_WARN("exiting hw main");
  return 0;
}
