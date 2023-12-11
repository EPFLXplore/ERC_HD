cmake_minimum_required(VERSION 3.8)
project(fake_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
moveit_package()
find_package(trajectory_planner)
find_package(kerby_moveit_config)
find_package(hd_interfaces)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
  rclpy
  moveit_common
  moveit_ros_planning_interface
  control_msgs
  moveit_servo
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# install launch directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


# include Cpp "include" directory
include_directories(include)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables

install(PROGRAMS
  scripts/fake_cs_gamepad.py
  scripts/fake_cs_keyboard.py
  scripts/fake_motor_control.py
  scripts/fake_cs_task.py
  scripts/fake_vision.py
  scripts/vision_test.py
  scripts/test_gamepad.py
  scripts/fake_rover_control.py
  scripts/limbs_control.py
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(servoing_test src/servoing_test.cpp)
ament_target_dependencies(servoing_test
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  control_msgs
  hd_interfaces
  moveit_servo
)
install(TARGETS servoing_test
  DESTINATION lib/${PROJECT_NAME})




# Add executable for using a controller
set(SERVO_CONTROLLER_INPUT servo_controller_input)
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/servoing_test.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "fake_components::JoyToServoPub")

# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
