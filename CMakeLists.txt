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
# Setup the path to load CMake macros, and extra CMake files.
######################################################################
set(CMAKE_MODULE_PATH
    ${CMAKE_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
   )

######################################################################
# Build options.
######################################################################
option(BUILD_TESTING "Build Unit tests." ON)

option(BUILD_SHARED_LIBS "Build Shared Libraries" ON)
if(WIN32 AND NOT BUILD_SHARED_LIBS)
  add_definitions(-DNIFTYCAL_STATIC)
endif()


######################################################################
# Configure Dart testing support.  This should be done before any
# message(FATAL_ERROR ...) commands are invoked.
######################################################################
include(${CMAKE_ROOT}/Modules/Dart.cmake)
mark_as_advanced(TCL_TCLSH DART_ROOT)
enable_testing()

if(BUILD_TESTING)
  set(BUILDNAME "NiftyCal" CACHE STRING "Name of build on the dashboard")
  mark_as_advanced(BUILDNAME)

  # Setup file for setting custom ctest vars
  configure_file(CMake/CTestCustom.cmake.in ${CMAKE_BINARY_DIR}/CTestCustom.cmake @ONLY)

  # Setup continuous test script
  configure_file(CMake/CTestContinuous.cmake.in ${CMAKE_BINARY_DIR}/CTestContinuous.cmake @ONLY)

endif(BUILD_TESTING)

######################################################################
# NiftyCal uses KWStyle for checking the coding style
######################################################################
include(${CMAKE_SOURCE_DIR}/Utilities/KWStyle/NiftyCalKWStyle.cmake)

######################################################################
# NiftyCal uses CppCheck for static analysis
######################################################################
include(${CMAKE_SOURCE_DIR}/Utilities/CppCheck/NiftyCalCppCheck.cmake)

######################################################################
# Add Mandatory Requirements
######################################################################
set(ALL_LIBRARIES niftycal)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_LIBRARIES ${OpenCV_LIBS})

######################################################################
# Add Optional Requirements
######################################################################
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindAprilTags.cmake ${CMAKE_BINARY_DIR}/FindAprilTags.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindEigen.cmake ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY IMMEDIATE)

find_package(Eigen)
if(Eigen_FOUND)
  include_directories(${Eigen_INCLUDE_DIR})

  find_package(AprilTags)
  if(AprilTags_FOUND)
    include_directories(${AprilTags_INCLUDE_DIR})
    list(APPEND ALL_LIBRARIES ${AprilTags_LIBRARIES})
  endif()
endif()

######################################################################
# Provide support for manually running Doxygen in build folder.
######################################################################
configure_file(${CMAKE_SOURCE_DIR}/Utilities/Doxygen/niftkdoxygen.pl.in ${CMAKE_BINARY_DIR}/niftkdoxygen.pl)
configure_file(${CMAKE_SOURCE_DIR}/Utilities/Doxygen/doxygen.config.in ${CMAKE_BINARY_DIR}/doxygen.config)

######################################################################
# Add code folders.
######################################################################
include_directories(${CMAKE_SOURCE_DIR}/Code/)
add_subdirectory(Code)
if(BUILD_TESTING)
  set(TEMP_DIR ${CMAKE_BINARY_DIR}/Testing/Temporary)
  include_directories(${CMAKE_SOURCE_DIR}/Testing/)
  add_subdirectory(Testing)
endif()

##################################################################################
# Generate "Use" and "Configure" files so external projects can link to NiftyCal.
# These go at or near the end, as all the variables need to be defined.
##################################################################################

configure_file(${CMAKE_SOURCE_DIR}/LICENSE.txt ${CMAKE_BINARY_DIR}/LICENSE.txt @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/README.md ${CMAKE_BINARY_DIR}/README.md @ONLY)

set(NiftyCal_CFG_DIR)
if(NOT CMAKE_CFG_INTDIR STREQUAL ".")
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(NiftyCal_CFG_DIR "Release/")
  endif()
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(NiftyCal_CFG_DIR "Debug/")
  endif()
endif()

configure_file(${CMAKE_SOURCE_DIR}/UseNiftyCal.cmake.in ${CMAKE_BINARY_DIR}/UseNiftyCal.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/NiftyCalConfig.cmake.in ${CMAKE_BINARY_DIR}/NiftyCalConfig.cmake @ONLY IMMEDIATE)


