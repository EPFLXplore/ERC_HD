cmake_minimum_required(VERSION 3.16)
project(ethercat_device_configurator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++1z -Wno-error)
endif()


set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(message_logger REQUIRED)
find_package(soem_interface REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(maxon_epos_ethercat_sdk REQUIRED)
find_package(motor_control_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(soem REQUIRED)
add_executable(motor_control  src/${PROJECT_NAME}/motor_control.cpp src/${PROJECT_NAME}/EthercatDeviceConfigurator.cpp)

# # Set the path to the third-party library folder
# set(OSHW include/ethercat_device_configurator/soem/oshw)
# set(OSAL include/ethercat_device_configurator/soem/osal)


# # Find all .cpp files in the third-party library folder

include_directories( 
  include/
  include/ethercat_device_configurator
  yaml-cpp/include/
  ${soem_interface_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}
  ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
  ${message_logger_INCLUDE_DIRS}
  ${ethercat_sdk_master_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)




ament_target_dependencies(motor_control
  yaml-cpp
  motor_control_interfaces #rclcpp  
  rclcpp
  ethercat_sdk_master
  maxon_epos_ethercat_sdk
  message_logger
  soem_interface
  soem
  std_msgs
  sensor_msgs
)

#target_link_directories(motor_control motor_control_interfaces)

target_link_libraries(motor_control
    Threads::Threads
    ${catkin_LIBRARIES}
  #   motor_control_interfaces
    maxon_epos_ethercat_sdk::mxn_sdk_lib
    yaml-cpp
    soem::soemrsl_lib
    soem_interface::soem_lib
    ethercat_sdk_master::eth_sdk_lib
    #  maxon_epos_ethercat_sdk
    #  message_logger
    #  soem_interface
     #soem
)


rosidl_generate_interfaces(${PROJECT_NAME} 
  "../motor_control_interfaces/msg/MotorCommand.msg"
  "../motor_control_interfaces/msg/MotorData.msg"
 )

install(TARGETS motor_control
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()


ament_package()


