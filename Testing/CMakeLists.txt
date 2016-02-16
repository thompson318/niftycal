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
set(TEST_CASES
  niftkNiftyCalExceptionTest
  niftkExtractChessboardPointsTest
)
foreach(_test_case ${TEST_CASES})
  add_executable(${_test_case} ${_test_case}.cxx niftkCatchMain.cxx)
  target_link_libraries(${_test_case} ${ALL_LIBRARIES})
endforeach()

# Adding tests is done outside the above loop,
# as each test will need different arguments.

##################################################################################################
# Dont forget its:  add_test(<test name (globally unique) > <exe name> <argument1> <argument2>
##################################################################################################

add_test(NiftyCalExcept niftkNiftyCalExceptionTest)
add_test(ExtChessZhang1 niftkExtractChessboardPointsTest ${DATA_DIR}/OpenCV-2.4.11/left01.png 640 480 9 6)

