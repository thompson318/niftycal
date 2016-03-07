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
  niftkPointDistortionTest
  niftkExtractChessboardPointsTest
  niftkExtractCirclesPointsTest
  niftkWarpChessboardByHomographyTest
  niftkMonoChessboardCameraCalibrationTest
  niftkIterativeMonoChessboardCameraCalibrationTest
  niftkStereoChessboardCameraCalibrationTest
  niftkIterativeStereoChessboardCameraCalibrationTest
  niftkIterativeStereoCirclesCameraCalibrationTest
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
add_test(ExtCircles1 niftkExtractCirclesPointsTest ${DATA_DIR}/SimulationDots1/Left1.jpg 1280 989 11 4)
add_test(PointDist niftkPointDistortionTest ${DATA_DIR}/Laparoscope/calib.left.intrinsic.txt ${DATA_DIR}/Laparoscope/calib.left.distortion.txt 1920 1080 1098.3 267.087)
add_test(WarpHomog1 niftkWarpChessboardByHomographyTest ${DATA_DIR}/Laparoscope/Left/left-1095.png ${DATA_DIR}/Laparoscope/Left/left-1338.png 14 10 1.0880866642) #RMS error of difference in points.

# For Laparoscope dataset, gives the benchmark for mono calibration.
add_test(CalibLapMonoLeft niftkMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  0.577703                                       # RMS
  2007.33 2013.72 945.486 619.07                 # intrinsic
  -0.237351 -0.244029 0.00299158 -0.00141141     # distortion
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
add_test(CalibLapMonoRight niftkMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  0.610067                                       # RMS
  2030.46 2046.25 1046.24 552.077                # intrinsic
  -0.20807 -0.497051 -0.00654118 0.000343286     # distortion
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

# For Laparoscope dataset, the stereo should give very close
# results for both left and right channels, as it did for the mono.
# This can be check in log file, rather than have too many program parameters.
# Here, we just check the stereo transform.
add_test(CalibLapStereo niftkStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  0.99919 -0.000809931 0.0402285                 # rvec
  -4.64005 0.26403 0.297868                      # tvec
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

# Now try iterative. Using simulation. No distortion.
add_test(CalibIterMonoSim2 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  480 270                                               # size of reference image
  ${DATA_DIR}/Simulation2/reference_480_270_14_10.txt   # 2D points for reference image
  1                                                     # zero distortion (1==true)
  0.222406                                              # RMS
  1540.4 1536.42 641.863 478.554                        # intrinsic
  0 0 0 0                                               # distortion
  ${DATA_DIR}/Simulation2/Left1.jpg
  ${DATA_DIR}/Simulation2/Left2.jpg
  ${DATA_DIR}/Simulation2/Left3.jpg
  ${DATA_DIR}/Simulation2/Left4.jpg
  ${DATA_DIR}/Simulation2/Left5.jpg
  ${DATA_DIR}/Simulation2/Left6.jpg
  ${DATA_DIR}/Simulation2/Left7.jpg
  ${DATA_DIR}/Simulation2/Left8.jpg
)

# Now try iterative on stereo.
add_test(CalibIterStereoSim2 niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  480 270                                               # size of reference image
  ${DATA_DIR}/Simulation2/reference_480_270_14_10.txt   # 2D points for reference image
  1                                                     # zero distortion (1==true)
  0.996334 0.000453448 -0.0855442 # rvec
  5.98487 -0.000715996 0.504326 # tvec
  ${DATA_DIR}/Simulation2/Left1.jpg
  ${DATA_DIR}/Simulation2/Left2.jpg
  ${DATA_DIR}/Simulation2/Left3.jpg
  ${DATA_DIR}/Simulation2/Left4.jpg
  ${DATA_DIR}/Simulation2/Left5.jpg
  ${DATA_DIR}/Simulation2/Left6.jpg
  ${DATA_DIR}/Simulation2/Left7.jpg
  ${DATA_DIR}/Simulation2/Left8.jpg
  ${DATA_DIR}/Simulation2/Right1.jpg
  ${DATA_DIR}/Simulation2/Right2.jpg
  ${DATA_DIR}/Simulation2/Right3.jpg
  ${DATA_DIR}/Simulation2/Right4.jpg
  ${DATA_DIR}/Simulation2/Right5.jpg
  ${DATA_DIR}/Simulation2/Right6.jpg
  ${DATA_DIR}/Simulation2/Right7.jpg
  ${DATA_DIR}/Simulation2/Right8.jpg
)

# Check iterative does something on laparoscope.
add_test(CalibLapLeftIter niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  960 540                                               # size of reference image
  ${DATA_DIR}/Laparoscope/reference_960_540_14_10.txt   # 2D points for reference image
  0                                                     # zero distortion (0==false)
  0.588097                                              # RMS
  2004.98 2012.29 945.409 621.189                       # intrinsic
  -0.237534 -0.212468 0.00267685 -0.00113922            # distortion
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
add_test(CalibLapStereoIter niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  960 540                                               # size of reference image
  ${DATA_DIR}/Laparoscope/reference_960_540_14_10.txt   # 2D points for reference image
  0                                                     # zero distortion (1==true)
  0.999096 -0.00102168 0.0424963 # rvec
  -4.68027 0.272581 0.148799 # tvec
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


# Now try iterative on circles.
add_test(CalibIterStereoCircles niftkIterativeStereoCirclesCameraCalibrationTest
  ${DATA_DIR}/SimulationDots1/circles_11_4_3.txt           # Model description (chessboard points)
  11 4                                                     # pattern size
  1280 989                                                 # size of reference image
  ${DATA_DIR}/SimulationDots1/reference_1280_989_11_4.txt  # 2D points for reference image
  1                                                        # zero distortion (1==true)
  0.995869 0.00066861 -0.0907979 # rvec
  2.43657 -0.000460828 0.262795 # tvec
  ${DATA_DIR}/SimulationDots1/Left1.jpg
  ${DATA_DIR}/SimulationDots1/Left2.jpg
  ${DATA_DIR}/SimulationDots1/Left3.jpg
  ${DATA_DIR}/SimulationDots1/Left4.jpg
  ${DATA_DIR}/SimulationDots1/Left5.jpg
  ${DATA_DIR}/SimulationDots1/Left6.jpg
  ${DATA_DIR}/SimulationDots1/Left7.jpg
  ${DATA_DIR}/SimulationDots1/Left8.jpg
  ${DATA_DIR}/SimulationDots1/Left9.jpg
  ${DATA_DIR}/SimulationDots1/Right1.jpg
  ${DATA_DIR}/SimulationDots1/Right2.jpg
  ${DATA_DIR}/SimulationDots1/Right3.jpg
  ${DATA_DIR}/SimulationDots1/Right4.jpg
  ${DATA_DIR}/SimulationDots1/Right5.jpg
  ${DATA_DIR}/SimulationDots1/Right6.jpg
  ${DATA_DIR}/SimulationDots1/Right7.jpg
  ${DATA_DIR}/SimulationDots1/Right8.jpg
  ${DATA_DIR}/SimulationDots1/Right9.jpg
)
