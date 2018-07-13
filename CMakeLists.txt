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

project(NiftyCal VERSION 1.0.3)

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
# Add Mandatory Requirements
######################################################################
set(ALL_THIRD_PARTY_LIBRARIES)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_THIRD_PARTY_LIBRARIES ${OpenCV_LIBS})

######################################################################
# Add Optional Requirements
######################################################################
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindAprilTags.cmake ${CMAKE_BINARY_DIR}/FindAprilTags.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/CMake/FindEigen.cmake ${CMAKE_BINARY_DIR}/FindEigen.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/Documentation/License.dox ${CMAKE_BINARY_DIR}/Doxygen/License.dox)
configure_file(${CMAKE_SOURCE_DIR}/Documentation/Licenses/OpenCV.txt ${CMAKE_BINARY_DIR}/LICENSE_OpenCV.txt)
configure_file(${CMAKE_SOURCE_DIR}/Documentation/Licenses/AprilTags.txt ${CMAKE_BINARY_DIR}/LICENSE_AprilTags.txt)
configure_file(${CMAKE_SOURCE_DIR}/Documentation/Licenses/Eigen.txt ${CMAKE_BINARY_DIR}/LICENSE_Eigen.txt)
configure_file(${CMAKE_SOURCE_DIR}/Documentation/Licenses/ITK.txt ${CMAKE_BINARY_DIR}/LICENSE_ITK.txt)
configure_file(${CMAKE_SOURCE_DIR}/Utilities/Doxygen/GenerateTestingReports.sh.in ${CMAKE_BINARY_DIR}/GenerateTestingReports.sh @ONLY )

find_package(Eigen)
if(Eigen_FOUND)
  include_directories(${Eigen_INCLUDE_DIR})

  find_package(AprilTags)
  if(AprilTags_FOUND)
    include_directories(${AprilTags_INCLUDE_DIR})
    list(APPEND ALL_THIRD_PARTY_LIBRARIES ${AprilTags_LIBRARIES})
  endif()
endif()

find_package(ITK COMPONENTS ITKOptimizers)
if(ITK_FOUND)
  include_directories(${ITK_INCLUDE_DIRS})
  list(APPEND ALL_THIRD_PARTY_LIBRARIES ${ITK_LIBRARIES})
  add_definitions(-DNIFTYCAL_WITH_ITK)
endif()

find_package(VTK)
if (VTK_FOUND)
  include(${VTK_USE_FILE})
  include_directories(${VTK_INCLUDE_DIRS})
  list(APPEND ALL_THIRD_PARTY_LIBRARIES ${VTK_LIBRARIES})
  add_definitions(-DNIFTYCAL_WITH_VTK)
endif()

######################################################################
# Find Qt.
######################################################################
set(DESIRED_QT_VERSION 4 CACHE STRING "Pick a version of Qt to use: 4 or 5")
set(QT5_LINK_LIBRARIES)
if(DESIRED_QT_VERSION MATCHES 4)
  find_package(Qt4 COMPONENTS QtCore QtGui)
  if(QT_FOUND)
    include(${QT_USE_FILE})
    list(APPEND ${ALL_THIRD_PARTY_LIBRARIES} ${QT_LIBRARIES})
  endif()
elseif(DESIRED_QT_VERSION MATCHES 5)
  set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE PATH "")
  set(_niftycal_qt_components Core Gui)
  find_package(Qt5 COMPONENTS ${_niftylink_qt_components})
  if(Qt5_DIR)
    get_filename_component(_Qt5_DIR "${Qt5_DIR}/../../../" ABSOLUTE)
    list(FIND CMAKE_PREFIX_PATH "${_Qt5_DIR}" _result)
    if(_result LESS 0)
      set(CMAKE_PREFIX_PATH "${_Qt5_DIR};${CMAKE_PREFIX_PATH}" CACHE PATH "" FORCE)
    endif()
    foreach(_component ${_niftylink_qt_components})
      find_package(Qt5${_component} REQUIRED QUIET)
      include_directories(${Qt5${_component}_INCLUDE_DIRS})
      add_definitions(${Qt5${_component}_DEFINITIONS})
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5${_component}_EXECUTABLE_COMPILE_FLAGS}")
      list(APPEND QT5_LINK_LIBRARIES Qt5::${_component})
    endforeach()
    list(APPEND ${ALL_THIRD_PARTY_LIBRARIES} ${QT5_LINK_LIBRARIES})
  endif()
else()
  message(FATAL "Invalid Qt version: ${DESIRED_QT_VERSION}. It should be only a '4' or a '5'")
endif()

set(ALL_LIBRARIES niftycal ${ALL_THIRD_PARTY_LIBRARIES})

find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

######################################################################
# Provide support for manually running Doxygen in build folder.
######################################################################
configure_file(${CMAKE_SOURCE_DIR}/Utilities/Doxygen/niftkdoxygen.pl.in ${CMAKE_BINARY_DIR}/niftkdoxygen.pl)
configure_file(${CMAKE_SOURCE_DIR}/Utilities/Doxygen/doxygen.config.in ${CMAKE_BINARY_DIR}/doxygen.config)

######################################################################
# NiftyCal uses KWStyle for checking the coding style
######################################################################
include(${CMAKE_SOURCE_DIR}/Utilities/KWStyle/NiftyCalKWStyle.cmake)

######################################################################
# NiftyCal uses CppCheck for static analysis
######################################################################
include(${CMAKE_SOURCE_DIR}/Utilities/CppCheck/NiftyCalCppCheck.cmake)

######################################################################
# Add code folders.
######################################################################
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
include_directories(${CMAKE_SOURCE_DIR}/Code/)
include_directories(${CMAKE_SOURCE_DIR}/Code/Detectors)
include_directories(${CMAKE_SOURCE_DIR}/Code/Optimisers)
include_directories(${CMAKE_SOURCE_DIR}/Code/IntensityBased)
add_subdirectory(Code)
add_subdirectory(Documentation)
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
export(TARGETS niftycal FILE "${CMAKE_BINARY_DIR}/NiftyCalTargets.cmake")
configure_file(${CMAKE_SOURCE_DIR}/NiftyCalConfig.cmake.in ${CMAKE_BINARY_DIR}/NiftyCalConfig.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/NiftyCalConfigVersion.cmake.in ${CMAKE_BINARY_DIR}/NiftyCalConfigVersion.cmake @ONLY IMMEDIATE)

