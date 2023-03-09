# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_astra_moveit_config_new_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED astra_moveit_config_new_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(astra_moveit_config_new_FOUND FALSE)
  elseif(NOT astra_moveit_config_new_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(astra_moveit_config_new_FOUND FALSE)
  endif()
  return()
endif()
set(_astra_moveit_config_new_CONFIG_INCLUDED TRUE)

# output package information
if(NOT astra_moveit_config_new_FIND_QUIETLY)
  message(STATUS "Found astra_moveit_config_new: 0.3.0 (${astra_moveit_config_new_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'astra_moveit_config_new' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${astra_moveit_config_new_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(astra_moveit_config_new_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${astra_moveit_config_new_DIR}/${_extra}")
endforeach()
