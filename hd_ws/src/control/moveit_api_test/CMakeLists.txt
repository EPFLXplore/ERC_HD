cmake_minimum_required(VERSION 3.8)
project(moveit_api_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(moveit_test src/test.cpp)
ament_target_dependencies(moveit_test rclcpp std_msgs moveit_core moveit_ros_planning_interface)

install(TARGETS
  moveit_test
  DESTINATION lib/${PROJECT_NAME})


add_executable(moveit_test2 src/test2.cpp)
ament_target_dependencies(moveit_test2 rclcpp std_msgs moveit_core moveit_ros_planning_interface)

install(TARGETS
  moveit_test2
  DESTINATION lib/${PROJECT_NAME})


add_executable(moveit_test3 src/test3.cpp)
ament_target_dependencies(moveit_test3 rclcpp std_msgs moveit_core moveit_ros_planning_interface rviz_visual_tools)

install(TARGETS
  moveit_test3
  DESTINATION lib/${PROJECT_NAME})


add_executable(motion_planning_test src/motion_planning.cpp)
ament_target_dependencies(motion_planning_test rclcpp std_msgs moveit_core moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools)

install(TARGETS
motion_planning_test
  DESTINATION lib/${PROJECT_NAME})


add_executable(move_group_interface_test src/move_group_interface_test.cpp)
ament_target_dependencies(move_group_interface_test rclcpp std_msgs moveit_core moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools)

install(TARGETS
move_group_interface_test
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
