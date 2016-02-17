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
  niftkMonoChessboardCameraCalibrationTest
)
if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND TEST_CASES niftkExtractAprilTagsPointsTest)
endif()

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
add_test(ExtAprilTags1 niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 792 612 35)
add_test(CalibMonoLap1 niftkMonoChessboardCameraCalibrationTest 3 14 10 0.577703 2007.33 2013.72 945.486 619.07 -0.237351 -0.244029 0.00299158 -0.00141141 ${DATA_DIR}/Laparoscope/left-1095.png	${DATA_DIR}/Laparoscope/left-1338.png	${DATA_DIR}/Laparoscope/left-1884.png	${DATA_DIR}/Laparoscope/left-2520.png	${DATA_DIR}/Laparoscope/left-3083.png	${DATA_DIR}/Laparoscope/left-3335.png	${DATA_DIR}/Laparoscope/left-3685.png	${DATA_DIR}/Laparoscope/left-4848.png	${DATA_DIR}/Laparoscope/left-798.png)
