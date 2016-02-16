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

set(DATA_DIR ${CMAKE_SOURCE_DIR}/Testing/Data)
file(GLOB CPP_FILES *.cxx)
foreach(_cpp_file ${CPP_FILES})
  get_filename_component(_cpp_file_we ${_cpp_file} NAME_WE)
  add_executable(${_cpp_file_we} ${_cpp_file})
  target_link_libraries(${_cpp_file_we} ${ALL_LIBRARIES})
endforeach()

# Adding tests is done outside the above loop,
# as each test will need different arguments.

##################################################################################################
# Dont forget its:  add_test(<test name (globally unique) > <exe name> <argument1> <argument2>
##################################################################################################

add_test(ExtChessZhang1 niftkExtractChessboardPoints ${DATA_DIR}/Zhang2000/CalibIm1.png 640 480 14 14)

