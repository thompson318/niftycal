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
  niftkStereoChessboardCameraCalibrationTest
  niftkWarpChessboardByHomographyTest
  niftkIterativeMonoChessboardCameraCalibrationTest
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
add_test(CalibMonoLap1 niftkMonoChessboardCameraCalibrationTest 3 14 10 0.577703 # RMS
  2007.33 2013.72 945.486 619.07              # intrinsic
  -0.237351 -0.244029 0.00299158 -0.00141141  # distortion
  ${DATA_DIR}/Laparoscope/Left/left-1095.png
  ${DATA_DIR}/Laparoscope/Left/left-1338.png
  ${DATA_DIR}/Laparoscope/Left/left-1884.png
  ${DATA_DIR}/Laparoscope/Left/left-2520.png
  ${DATA_DIR}/Laparoscope/Left/left-3083.png
  ${DATA_DIR}/Laparoscope/Left/left-3335.png
  ${DATA_DIR}/Laparoscope/Left/left-3685.png
  ${DATA_DIR}/Laparoscope/Left/left-4848.png
  ${DATA_DIR}/Laparoscope/Left/left-798.png
)
add_test(CalibStereoLap1 niftkStereoChessboardCameraCalibrationTest 3 14 10
  0.99919 -0.000809931 0.0402285 # rvec
  -4.64005 0.26403 0.297868 # tvec
  ${DATA_DIR}/Laparoscope/Left/left-1095.png
  ${DATA_DIR}/Laparoscope/Left/left-1338.png
  ${DATA_DIR}/Laparoscope/Left/left-1884.png
  ${DATA_DIR}/Laparoscope/Left/left-2520.png
  ${DATA_DIR}/Laparoscope/Left/left-3083.png
  ${DATA_DIR}/Laparoscope/Left/left-3335.png
  ${DATA_DIR}/Laparoscope/Left/left-3685.png
  ${DATA_DIR}/Laparoscope/Left/left-4848.png
  ${DATA_DIR}/Laparoscope/Left/left-798.png
  ${DATA_DIR}/Laparoscope/Right/right-1095.png
  ${DATA_DIR}/Laparoscope/Right/right-1338.png
  ${DATA_DIR}/Laparoscope/Right/right-1884.png
  ${DATA_DIR}/Laparoscope/Right/right-2520.png
  ${DATA_DIR}/Laparoscope/Right/right-3083.png
  ${DATA_DIR}/Laparoscope/Right/right-3335.png
  ${DATA_DIR}/Laparoscope/Right/right-3685.png
  ${DATA_DIR}/Laparoscope/Right/right-4848.png
  ${DATA_DIR}/Laparoscope/Right/right-798.png
)
add_test(WarpHomog1 niftkWarpChessboardByHomographyTest ${DATA_DIR}/Laparoscope/Left/left-1095.png ${DATA_DIR}/Laparoscope/Left/left-1338.png 14 10 1.0880866642) #RMS error of difference in points.
add_test(CalibIterativeMonoLap1 niftkIterativeMonoChessboardCameraCalibrationTest 3 14 10
  960 540                                               # size of reference image
  ${DATA_DIR}/Laparoscope/reference_960_540_14_10.txt   # 2D points for reference image
  0.577703                                              # RMS
  2007.33 2013.72 945.486 619.07                        # intrinsic
  -0.237351 -0.244029 0.00299158 -0.00141141            # distortion
  ${DATA_DIR}/Laparoscope/Left/left-1095.png
  ${DATA_DIR}/Laparoscope/Left/left-1338.png
  ${DATA_DIR}/Laparoscope/Left/left-1884.png
  ${DATA_DIR}/Laparoscope/Left/left-2520.png
  ${DATA_DIR}/Laparoscope/Left/left-3083.png
  ${DATA_DIR}/Laparoscope/Left/left-3335.png
  ${DATA_DIR}/Laparoscope/Left/left-3685.png
  ${DATA_DIR}/Laparoscope/Left/left-4848.png
  ${DATA_DIR}/Laparoscope/Left/left-798.png
)
