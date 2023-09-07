cmake_minimum_required(VERSION 3.5)
project(coucou)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor_control_interfaces REQUIRED)

add_executable(talker src/send.cpp)
ament_target_dependencies(talker rclcpp std_msgs motor_control_interfaces)

add_executable(suscriber src/read.cpp)
ament_target_dependencies(suscriber rclcpp sensor_msgs motor_control_interfaces)

add_executable(suscriber_test src/read_test.cpp)
ament_target_dependencies(suscriber_test rclcpp motor_control_interfaces)


install(TARGETS
  talker
  suscriber
  suscriber_test
  DESTINATION lib/${PROJECT_NAME})


ament_package()