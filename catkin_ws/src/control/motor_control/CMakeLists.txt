cmake_minimum_required(VERSION 3.0.2)
project(motor_control)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
message_filters
message_generation
)

add_message_files(
 FILES
 simJointState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
CATKIN_DEPENDS rospy roscpp message_runtime
)


include_directories(
# include
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
)

message(STATUS "DIR : --------->  ${PROJECT_SOURCE_DIR}")


add_executable(motor_control src/motor_main0.cpp
        include/xcontrol_v2/epos4_extended.h
        include/xcontrol_v2/network_master.h
        include/xcontrol_v2/one_axis_slot.h
        include/xcontrol_v2/three_axis_slot.h
        lib/xcontrol_v2/epos4_extended.cpp
        lib/xcontrol_v2/network_master.cpp
        lib/xcontrol_v2/one_axis_slot.cpp
        lib/xcontrol_v2/three_axis_slot.cpp
        )

target_link_libraries(motor_control ${catkin_LIBRARIES} 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so 
                    ${PROJECT_SOURCE_DIR}/lib/libpid-os-utilities_pid-signal-manager.so 
                    ${PROJECT_SOURCE_DIR}/lib/libsoem.a)


add_dependencies(motor_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




add_executable(testt src/test.cpp
        include/xcontrol_v2/epos4_extended.h
        include/xcontrol_v2/network_master.h
        include/xcontrol_v2/one_axis_slot.h
        include/xcontrol_v2/three_axis_slot.h
        lib/xcontrol_v2/epos4_extended.cpp
        lib/xcontrol_v2/network_master.cpp
        lib/xcontrol_v2/one_axis_slot.cpp
        lib/xcontrol_v2/three_axis_slot.cpp
        )

target_link_libraries(testt ${catkin_LIBRARIES} 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so 
                    ${PROJECT_SOURCE_DIR}/lib/libpid-os-utilities_pid-signal-manager.so 
                    ${PROJECT_SOURCE_DIR}/lib/libsoem.a)


add_dependencies(testt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})