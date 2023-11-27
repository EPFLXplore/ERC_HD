cmake_minimum_required(VERSION 3.8)
project(maxon_epos_ethercat_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_logger REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(soem_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem REQUIRED)


#=========================================================================
#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
  ethercat_sdk_master
)

find_package(ament_cmake REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)
set(HEADER_FILES include/ethercat_sdk_master)
file(GLOB MXN_SDK_LIB ${HEADER_FILES}/*.hpp)


###########
## Build ##
###########

include_directories(
  include
  ${message_logger_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}
  ${soem_interface_INCLUDE_DIRS}
  ${ethercat_sdk_master_INCLUDE_DIRS}
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${MXN_SDK_LIB}
)

FILE(GLOB CPPSources src/${PROJECT_NAME}/*.cpp)
add_library(mxn_sdk_lib
  ${CPPSources}
  ${MXN_SDK_LIB}
)

ament_export_targets(mxn_sdk_lib HAS_LIBRARY_TARGET)

target_link_libraries(
  mxn_sdk_lib
  ethercat_sdk_master::eth_sdk_lib
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(mxn_sdk_lib
  "rclcpp"
  message_logger
  soem_interface
  ethercat_sdk_master
)

#############
## Install ##
#############

install(
  DIRECTORY include/maxon_epos_ethercat_sdk
  DESTINATION include
)

install(
  TARGETS mxn_sdk_lib
  EXPORT mxn_sdk_lib
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
