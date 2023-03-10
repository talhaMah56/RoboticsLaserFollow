# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Zelda_py_laserfollow_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Zelda_py_laserfollow_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Zelda_py_laserfollow_FOUND FALSE)
  elseif(NOT Zelda_py_laserfollow_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Zelda_py_laserfollow_FOUND FALSE)
  endif()
  return()
endif()
set(_Zelda_py_laserfollow_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Zelda_py_laserfollow_FIND_QUIETLY)
  message(STATUS "Found Zelda_py_laserfollow: 0.0.0 (${Zelda_py_laserfollow_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Zelda_py_laserfollow' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Zelda_py_laserfollow_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Zelda_py_laserfollow_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Zelda_py_laserfollow_DIR}/${_extra}")
endforeach()
