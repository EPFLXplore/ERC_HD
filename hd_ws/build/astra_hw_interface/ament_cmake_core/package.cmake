set(_AMENT_PACKAGE_NAME "astra_hw_interface")
set(astra_hw_interface_VERSION "0.0.0")
set(astra_hw_interface_MAINTAINER "matthias <matthias.schuller@ulb.ac.be>")
set(astra_hw_interface_BUILD_DEPENDS "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(astra_hw_interface_BUILDTOOL_DEPENDS "ament_cmake")
set(astra_hw_interface_BUILD_EXPORT_DEPENDS "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(astra_hw_interface_BUILDTOOL_EXPORT_DEPENDS )
set(astra_hw_interface_EXEC_DEPENDS "ros2_controllers_test_nodes" "joint_state_broadcaster" "forward_command_controller" "joint_trajectory_controller" "ros2controlcli" "controller_manager" "rviz2" "robot_state_publisher" "hardware_interface" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(astra_hw_interface_TEST_DEPENDS "ament_lint_auto" "ament_lint_common" "ament_add_gmock" "hardware_interface")
set(astra_hw_interface_GROUP_DEPENDS )
set(astra_hw_interface_MEMBER_OF_GROUPS )
set(astra_hw_interface_DEPRECATED "")
set(astra_hw_interface_EXPORT_TAGS)
list(APPEND astra_hw_interface_EXPORT_TAGS "<build_type>ament_cmake</build_type>")