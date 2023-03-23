cmake_minimum_required(VERSION 3.8)
project(astra_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(astra_hw_interface SHARED
    src/astra_arm_interface.cpp
)
target_compile_features(astra_hw_interface PUBLIC cxx_std_17)
# target_include_directories(astra_hw_interface PUBLIC
#     include
#     include/astra_hw_interface)
target_include_directories(astra_hw_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/astra_hw_interface>
)
ament_target_dependencies(
  astra_hw_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ASTRA_HW_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface astra_hw_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/astra_hw_interface
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf description/rviz
#   DESTINATION share/ros2_control_demo_example_1
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/ros2_control_demo_example_1
# )
install(TARGETS astra_hw_interface
  EXPORT astra_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)

  # ament_add_gmock(test_load_astra_arm_interface test/test_load_astra_arm_interface.cpp)
  # target_include_directories(test_load_astra_arm_interface PRIVATE include)
  # ament_target_dependencies(test_load_astra_arm_interface rcpputils)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



## EXPORTS
ament_export_targets(astra_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
