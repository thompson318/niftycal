#/*============================================================================
#
#  NiftyCal: A software package for camera calibration.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

######################################################################
# Set the minimum CMake version.
######################################################################
set(NIFTYCAL_CMAKE_MINIMUM_REQUIRED_VERSION 3.2)
cmake_minimum_required(VERSION ${NIFTYCAL_CMAKE_MINIMUM_REQUIRED_VERSION})

##################################################################################
# Set some CMake Policies.
# See http://cmake.org/cmake/help/cmake-2-8-docs.html#section_Policies for details
##################################################################################

set(project_policies )
foreach(policy ${project_policies})
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endforeach()

project(NIFTYCAL VERSION 00.00.1)

######################################################################
# Setting supported build types. Should ONLY be Release or Debug.
######################################################################

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Valid options are Release or Debug" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug")
endif()

if (NOT (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "Debug"))
  message(FATAL_ERROR "Build type \"${CMAKE_BUILD_TYPE}\" is not supported.")
endif()

if(WIN32)
  # Restrict the generated configuration to be what we configured above.
  # No point creating project files for build types that will not compile.
  # Note: it's set to FORCE so that both CMAKE_BUILD_TYPE and CMAKE_CONFIGURATION_TYPES match up.
  set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "Build configurations to generate." FORCE)
  mark_as_advanced(CMAKE_CONFIGURATION_TYPES)
endif()

######################################################################
# Build options.
######################################################################
option(BUILD_TESTING "Build Unit tests." OFF)

######################################################################
# Add Mandatory Requirements
######################################################################
find_package(OpenCV REQUIRED)
message("Matt, include:${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

######################################################################
# Add code folders.
######################################################################
include_directories(${CMAKE_SOURCE_DIR}/Code/include)
add_subdirectory(Code)
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()


