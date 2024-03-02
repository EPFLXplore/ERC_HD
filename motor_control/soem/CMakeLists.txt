cmake_minimum_required(VERSION 3.5)
project(soem)

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

find_package(ament_cmake REQUIRED)

# Add include directories
include_directories(
  include
  soem/include
)

set(HEADER_FILES include/soem)
file(GLOB SOEMH_LOG_LIB ${HEADER_FILES}/soem/*.hpp)
file(GLOB OSALH_LOG_LIB ${HEADER_FILES}/osal/*.hpp)
file(GLOB OSHWH_LOG_LIB ${HEADER_FILES}/oshw/*.hpp)
set(SOURCES_FILES src/soem)
file(GLOB SOEMC_LOG_LIB ${SOURCES_FILES}/soem/*.c)
file(GLOB OSALC_LOG_LIB ${SOURCES_FILES}/osal/*.c)
file(GLOB OSHWC_LOG_LIB ${SOURCES_FILES}/oshw/*.c)

# Create a shared library
add_library(soemrsl_lib SHARED
  ${SOEMH_LOG_LIB}
  ${OSALH_LOG_LIB}
  ${OSHWH_LOG_LIB}
  ${SOEMC_LOG_LIB}
  ${OSALC_LOG_LIB}
  ${OSHWC_LOG_LIB}
)

ament_export_targets(soemrsl_lib HAS_LIBRARY_TARGET)

install(DIRECTORY include/soem
  DESTINATION include
)

# Install library and header files
install(TARGETS soemrsl_lib
  EXPORT soemrsl_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add environment hooks

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
