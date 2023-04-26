cmake_minimum_required(VERSION 3.8)
project(kerby_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(moveit2_tutorials REQUIRED)
find_package(manager)
find_package(moveit_configs_utils)


## Install Python modules
#ament_python_install_package(${PROJECT_NAME})

## Install Python executables
#install(PROGRAMS
#  scripts/moveit_configs_utils.py
#  DESTINATION lib/${PROJECT_NAME}
#)


ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
