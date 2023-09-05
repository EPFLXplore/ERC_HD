cmake_minimum_required(VERSION 3.5)
project(trajectory_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kerby_interfaces)


# install launch directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


# include Cpp "include" directory
include_directories(include)



# executables

add_executable(planner src/planner_main.cpp src/planner.cpp)
# target_include_directories(planner PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(planner
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  kerby_interfaces
)

install(TARGETS planner
  DESTINATION lib/${PROJECT_NAME})



# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(kinematics_utils)
ament_python_install_package(task_execution)




# Install Python executables

install(PROGRAMS
  scripts/task_executor.py
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
