# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kuka_control_box_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kuka_control_box_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kuka_control_box_FOUND FALSE)
  elseif(NOT kuka_control_box_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kuka_control_box_FOUND FALSE)
  endif()
  return()
endif()
set(_kuka_control_box_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kuka_control_box_FIND_QUIETLY)
  message(STATUS "Found kuka_control_box: 0.0.0 (${kuka_control_box_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kuka_control_box' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${kuka_control_box_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kuka_control_box_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${kuka_control_box_DIR}/${_extra}")
endforeach()
