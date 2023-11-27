cmake_minimum_required(VERSION 3.5)
project(message_logger LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


# Configure header file
configure_file(
  include/message_logger/log/log_messages_backend_config.hpp.in
  ${PROJECT_BINARY_DIR}/include/message_logger/log/log_messages_backend_config.hpp
)

# Add include directories
include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  include/message_logger
)

set(HEADER_FILES include/message_logger)
file(GLOB MSG_LOG_LIB ${HEADER_FILES}/*.hpp)


# Create a shared library
add_library(msg_lib SHARED
  src/time/Time.cpp
  src/time/TimeStd.cpp
  ${MSG_LOG_LIB}
)

ament_export_targets(msg_lib HAS_LIBRARY_TARGET)

# Link with ROS2 libraries
ament_target_dependencies(msg_lib
  "rclcpp"
  "std_msgs"
)

install(DIRECTORY include/message_logger
  DESTINATION include
)

# Install library and header files
install(TARGETS msg_lib
  EXPORT msg_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add environment hooks

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
