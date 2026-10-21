# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mujoco_micro_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mujoco_micro_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mujoco_micro_FOUND FALSE)
  elseif(NOT mujoco_micro_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mujoco_micro_FOUND FALSE)
  endif()
  return()
endif()
set(_mujoco_micro_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mujoco_micro_FIND_QUIETLY)
  message(STATUS "Found mujoco_micro: 1.0.0 (${mujoco_micro_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mujoco_micro' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mujoco_micro_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mujoco_micro_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${mujoco_micro_DIR}/${_extra}")
endforeach()
