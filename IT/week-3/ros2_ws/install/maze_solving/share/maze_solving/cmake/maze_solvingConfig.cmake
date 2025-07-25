# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_maze_solving_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED maze_solving_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(maze_solving_FOUND FALSE)
  elseif(NOT maze_solving_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(maze_solving_FOUND FALSE)
  endif()
  return()
endif()
set(_maze_solving_CONFIG_INCLUDED TRUE)

# output package information
if(NOT maze_solving_FIND_QUIETLY)
  message(STATUS "Found maze_solving: 0.0.0 (${maze_solving_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'maze_solving' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${maze_solving_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(maze_solving_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${maze_solving_DIR}/${_extra}")
endforeach()
