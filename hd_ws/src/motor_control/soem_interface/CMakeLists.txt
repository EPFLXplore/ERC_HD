cmake_minimum_required(VERSION 3.5)
project(soem_interface)

add_definitions(-std=c++14 -Wall)
add_definitions(-DMELO_USE_COUT)

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

set(HEADER_FILES include/soem_interface/EthercatBusBase.hpp 
     include/soem_interface/EthercatBusManagerBase.hpp
     include/soem_interface/EthercatSlaveBase.hpp
     include/soem_interface/common/ThreadSleep.hpp
     include/soem_interface/common/Macros.hpp)

     

find_package(ament_cmake REQUIRED)
find_package(message_logger REQUIRED)
find_package(soem REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${soem_INCLUDE_DIRS}
  ${message_logger_INCLUDE_DIRS}
  ${HEADER_FILES}
)


add_library(soem_lib 
  src/${PROJECT_NAME}/common/ThreadSleep.cpp
  src/${PROJECT_NAME}/common/Macros.cpp
  src/${PROJECT_NAME}/EthercatSlaveBase.cpp
  src/${PROJECT_NAME}/EthercatBusManagerBase.cpp
  src/${PROJECT_NAME}/EthercatBusBase.cpp
  ${HEADER_FILES}
  
)

ament_export_targets(soem_lib HAS_LIBRARY_TARGET)


target_link_libraries(soem_lib
  message_logger::msg_lib
  soem::soemrsl_lib)

ament_target_dependencies(soem_lib
  message_logger
)



install(
  DIRECTORY include/soem_interface
  DESTINATION include
)

install(
  TARGETS soem_lib
  EXPORT soem_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
