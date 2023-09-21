cmake_minimum_required(VERSION 3.8)
project(ethercat_sdk_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soem_interface REQUIRED)
find_package(message_logger REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#=========================================================================
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

# set(CATKIN_PACKAGE_DEPENDENCIES
#   soem_interface
# )

find_package(Threads REQUIRED)

set(HEADER_FILES include/ethercat_sdk_master)
file(GLOB ETH_SDK_GLOB_LIB ${HEADER_FILES}/*.hpp)

###########
## Build ##
###########
include_directories(
  include
  ${message_logger_INCLUDE_DIRS}
  ${soem_interface_INCLUDE_DIRS}
  ${ETH_SDK_GLOB_LIB}
  ${soem_INCLUDE_DIRS}
  include/ethercat_sdk_master
  SYSTEM
)

add_library(eth_sdk_lib
  src/${PROJECT_NAME}/EthercatMaster.cpp
  src/${PROJECT_NAME}/EthercatDevice.cpp
  ${ETH_SDK_GLOB_LIB}
)


ament_export_targets(eth_sdk_lib HAS_LIBRARY_TARGET)

target_link_libraries(eth_sdk_lib
  ${YAML_CPP_LIBRARIES}
  soem_interface::soem_lib
  Threads::Threads

)


ament_target_dependencies(eth_sdk_lib
  "rclcpp"
  soem_interface
  message_logger
)

#############
## Install ##
#############

install(
  DIRECTORY include/ethercat_sdk_master
  DESTINATION include
)

install(
  TARGETS eth_sdk_lib
  EXPORT eth_sdk_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#=========================================================================


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
