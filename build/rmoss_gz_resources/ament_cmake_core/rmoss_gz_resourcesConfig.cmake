# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rmoss_gz_resources_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rmoss_gz_resources_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rmoss_gz_resources_FOUND FALSE)
  elseif(NOT rmoss_gz_resources_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rmoss_gz_resources_FOUND FALSE)
  endif()
  return()
endif()
set(_rmoss_gz_resources_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rmoss_gz_resources_FIND_QUIETLY)
  message(STATUS "Found rmoss_gz_resources: 0.9.0 (${rmoss_gz_resources_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rmoss_gz_resources' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rmoss_gz_resources_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rmoss_gz_resources_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rmoss_gz_resources_DIR}/${_extra}")
endforeach()
