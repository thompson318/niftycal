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
  niftkIterativeStereoAprilTagsCameraCalibrationTest
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
add_test(ExtChessSim1 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png 1500 1100 14 10)
add_test(ExtChessSim2 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_750_550_14_10.png 750 550 14 10)
add_test(ExtChessSim3 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_375_275_14_10.png 375 275 14 10)
add_test(ExtChessZhang1 niftkExtractChessboardPointsTest ${DATA_DIR}/OpenCV-2.4.11/left01.png 640 480 9 6)
add_test(ExtChessCMU1 niftkExtractChessboardPointsTest ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp 720 480 10 7)
add_test(ExtCircles1 niftkExtractCirclesPointsTest ${DATA_DIR}/SimulationDots1/Left1.jpg 1280 1024 11 4)
add_test(ExtAprilTags1 niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 792 612 35)
add_test(ExtAprilTags2 niftkExtractAprilTagsPointsTest ${DATA_DIR}/SimulationAprilTags/Left1.jpg 1280 1024 35)
add_test(PointDist niftkPointDistortionTest ${DATA_DIR}/Laparoscope/calib.left.intrinsic.txt ${DATA_DIR}/Laparoscope/calib.left.distortion.txt 1920 1080 1098.3 267.087)
add_test(WarpHomog1 niftkWarpChessboardByHomographyTest ${DATA_DIR}/Laparoscope/Left/left-1095.png ${DATA_DIR}/Laparoscope/Left/left-1338.png 14 10 1.0880866642) #RMS error of difference in points.

# For Laparoscope dataset, gives the benchmark for standard OpenCV mono calibration for left camera.
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

# For Laparoscope dataset, gives the benchmark for standard OpenCV mono calibration for right camera.
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
# This can be checked in the log file, rather than have too many program parameters.
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

# Now try iterative. Using simulation 2. No distortion.
add_test(CalibIterMonoSim1 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model points
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.txt # 2D points for reference image
  1                                                     # zero distortion (1==true)
  0.276418                                              # RMS
  1750.43 1749.62 637.976 513.424                       # intrinsic (should be 1750, 1750, 639.5, 511.5)
  0 0 0 0                                               # distortion (should be 0, 0, 0, 0)
  ${DATA_DIR}/Simulation2/Left1.jpg
  ${DATA_DIR}/Simulation2/Left2.jpg
  ${DATA_DIR}/Simulation2/Left3.jpg
  ${DATA_DIR}/Simulation2/Left4.jpg
  ${DATA_DIR}/Simulation2/Left5.jpg
  ${DATA_DIR}/Simulation2/Left6.jpg
  ${DATA_DIR}/Simulation2/Left7.jpg
  ${DATA_DIR}/Simulation2/Left8.jpg
)

# Now try iterative. Using simulation 2. With distortion modelling, but should have no distortion.
add_test(CalibIterMonoSim2 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model points
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.txt # 2D points for reference image
  0                                                     # zero distortion (1==true)
  0.274544                                              # RMS
  1751.04 1749.8 639.982 512.216                        # intrinsic (should be 1750, 1750, 639.5, 511.5)
  -0.00224263 0.0110163 -0.000324203 0.000544618        # distortion (should be 0, 0, 0, 0)
  ${DATA_DIR}/Simulation2/Left1.jpg
  ${DATA_DIR}/Simulation2/Left2.jpg
  ${DATA_DIR}/Simulation2/Left3.jpg
  ${DATA_DIR}/Simulation2/Left4.jpg
  ${DATA_DIR}/Simulation2/Left5.jpg
  ${DATA_DIR}/Simulation2/Left6.jpg
  ${DATA_DIR}/Simulation2/Left7.jpg
  ${DATA_DIR}/Simulation2/Left8.jpg
)

# Now try iterative on stereo, using simulation 2.
add_test(CalibIterStereoSim1 niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model points
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.txt # 2D points for reference image
  1                                                     # zero distortion (1==true)
  0.996335 8.27736e-05 -0.0855332 # rvec
  5.98069 0.00132401 0.519151 # tvec
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
add_test(CalibIterLapLeft niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_750_550_14_10.png   # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_750_550_14_10.txt   # 2D points for reference image
  0                                                     # zero distortion (0==false)
  0.575338                                              # RMS, only marginally better than none iterative.
  2006.05 2012.63 948.229 621.751                       # intrinsic
  -0.224385 -0.358148 0.00282802 -0.00114772            # distortion
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

# Check iterative does something on stereo laparoscope.
# This currently fails. The problem is that the left and right
# camera get calibrated differently. Then warping them to
# the same reference image produces too high distortion on one channel,
# causing subsequent chessboard extraction to fail.
#add_test(CalibIterLapStereo niftkIterativeStereoChessboardCameraCalibrationTest
#  ${DATA_DIR}/Simulation2/reference_375_275_14_10.png   # Model image
#  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
#  14 10                                                 # internal corners
#  ${DATA_DIR}/Simulation2/reference_375_275_14_10.txt   # 2D points for reference image
#  0                                                     # zero distortion (1==true)
#  0.998899 -0.000514354 0.0469033 # rvec                # c.f. none iterative: 0.99919 -0.000809931 0.0402285
#  -4.62666 0.289513 0.450488 # tvec                     # c.f. none iterative: -4.64005 0.26403 0.297868
#  ${DATA_DIR}/Laparoscope/Left/left-1095.png
#  ${DATA_DIR}/Laparoscope/Left/left-1338.png
#  ${DATA_DIR}/Laparoscope/Left/left-1884.png
#  ${DATA_DIR}/Laparoscope/Left/left-2520.png
#  ${DATA_DIR}/Laparoscope/Left/left-3083.png
#  ${DATA_DIR}/Laparoscope/Left/left-3335.png
#  ${DATA_DIR}/Laparoscope/Left/left-3685.png
#  ${DATA_DIR}/Laparoscope/Left/left-4848.png
#  ${DATA_DIR}/Laparoscope/Left/left-798.png
#  ${DATA_DIR}/Laparoscope/Right/right-1095.png
#  ${DATA_DIR}/Laparoscope/Right/right-1338.png
#  ${DATA_DIR}/Laparoscope/Right/right-1884.png
#  ${DATA_DIR}/Laparoscope/Right/right-2520.png
#  ${DATA_DIR}/Laparoscope/Right/right-3083.png
#  ${DATA_DIR}/Laparoscope/Right/right-3335.png
#  ${DATA_DIR}/Laparoscope/Right/right-3685.png
#  ${DATA_DIR}/Laparoscope/Right/right-4848.png
#  ${DATA_DIR}/Laparoscope/Right/right-798.png
#)

# Now try iterative on circles.
# This currently fails. The problem is that the left and right
# camera get calibrated differently. Then warping them to
# the same reference image produces to high distortion on one channel,
# causing subsequent circle extraction to fail.
#add_test(CalibIterStereoCircles niftkIterativeStereoCirclesCameraCalibrationTest
#  ${DATA_DIR}/SimulationDots1/Left1.jpg                     # Model image
#  ${DATA_DIR}/SimulationDots1/circles_11_4_3.txt            # Model description (grid points 3D)
#  11 4                                                      # pattern size
#  ${DATA_DIR}/SimulationDots1/reference_1280_1024_11_4.txt  # 2D points for reference image
#  1                                                         # zero distortion (1==true)
#  0.995869 0.00066861 -0.0907979 # rvec
#  2.43657 -0.000460828 0.262795 # tvec
#  ${DATA_DIR}/SimulationDots1/Left1.jpg
#  ${DATA_DIR}/SimulationDots1/Left2.jpg
#  ${DATA_DIR}/SimulationDots1/Left3.jpg
#  ${DATA_DIR}/SimulationDots1/Left4.jpg
#  ${DATA_DIR}/SimulationDots1/Left5.jpg
#  ${DATA_DIR}/SimulationDots1/Left6.jpg
#  ${DATA_DIR}/SimulationDots1/Left7.jpg
#  ${DATA_DIR}/SimulationDots1/Left8.jpg
#  ${DATA_DIR}/SimulationDots1/Left9.jpg
#  ${DATA_DIR}/SimulationDots1/Right1.jpg
#  ${DATA_DIR}/SimulationDots1/Right2.jpg
#  ${DATA_DIR}/SimulationDots1/Right3.jpg
#  ${DATA_DIR}/SimulationDots1/Right4.jpg
#  ${DATA_DIR}/SimulationDots1/Right5.jpg
#  ${DATA_DIR}/SimulationDots1/Right6.jpg
#  ${DATA_DIR}/SimulationDots1/Right7.jpg
#  ${DATA_DIR}/SimulationDots1/Right8.jpg
#  ${DATA_DIR}/SimulationDots1/Right9.jpg
#)

# Now try iterative, mono, using CMU synthetic dataset.
add_test(CalibIterMonoCMU1 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp # Model image
  ${DATA_DIR}/CMU/TestSyntheticData/chessboard_10_7_3.txt                # Model description (chessboard points)
  10 7                                                                   # internal corners
  ${DATA_DIR}/CMU/TestSyntheticData/reference_720_480_10_7.txt           # 2D points for reference image
  0                                                                      # zero distortion (1==true)
  0.120261                                                              # RMS
  1148.16 1148.52 360.429 239.556                                        # intrinsic. Should be 1150, 1150, 360.5 240.5.
  -0.76516 1.96134 0.000388442 -0.000313784                             # distortion. Should be -0.7, -0.3, 0.004, 0.00005, 8.
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_-45_0_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_-45_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_45_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_45_0_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/1/checker_-45_-45_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/1/checker_-45_45_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/1/checker_0_0_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/1/checker_45_-45_0_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/1/checker_45_45_0_.bmp
)

# Now try iterative, mono, using CMU real dataset.
add_test(CalibIterMonoCMU2 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp # Model image
  ${DATA_DIR}/CMU/TestSyntheticData/chessboard_10_7_3.txt                # Model description (chessboard points)
  10 7                                                                   # internal corners
  ${DATA_DIR}/CMU/TestSyntheticData/reference_720_480_10_7.txt           # 2D points for reference image
  0                                                                      # zero distortion (1==true)
  0.15164                                                                # RMS
  842.226 841.219 507.523 367.148                                        # intrinsic
  -0.430247 0.254999 0.000639015 0.000501049                             # distortion
  ${DATA_DIR}/CMU/TestRealImage/checker/1/img1.bmp
  ${DATA_DIR}/CMU/TestRealImage/checker/1/img2.bmp
  ${DATA_DIR}/CMU/TestRealImage/checker/1/img3.bmp
  ${DATA_DIR}/CMU/TestRealImage/checker/1/img4.bmp
  ${DATA_DIR}/CMU/TestRealImage/checker/1/img5.bmp
)

# Now try iterative on AprilTags.
add_test(CalibIterStereoApril niftkIterativeStereoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  1                                                         # zero distortion (1==true)
  0.996357 -2.20494e-05 -0.0852827 # rvec
  5.97596 0.000262982 0.519377 # tvec
  ${DATA_DIR}/SimulationAprilTags/Left1.jpg
  ${DATA_DIR}/SimulationAprilTags/Left2.jpg
  ${DATA_DIR}/SimulationAprilTags/Left3.jpg
  ${DATA_DIR}/SimulationAprilTags/Left4.jpg
  ${DATA_DIR}/SimulationAprilTags/Left5.jpg
  ${DATA_DIR}/SimulationAprilTags/Left6.jpg
  ${DATA_DIR}/SimulationAprilTags/Left7.jpg
  ${DATA_DIR}/SimulationAprilTags/Left8.jpg
  ${DATA_DIR}/SimulationAprilTags/Left9.jpg
  ${DATA_DIR}/SimulationAprilTags/Right1.jpg
  ${DATA_DIR}/SimulationAprilTags/Right2.jpg
  ${DATA_DIR}/SimulationAprilTags/Right3.jpg
  ${DATA_DIR}/SimulationAprilTags/Right4.jpg
  ${DATA_DIR}/SimulationAprilTags/Right5.jpg
  ${DATA_DIR}/SimulationAprilTags/Right6.jpg
  ${DATA_DIR}/SimulationAprilTags/Right7.jpg
  ${DATA_DIR}/SimulationAprilTags/Right8.jpg
  ${DATA_DIR}/SimulationAprilTags/Right9.jpg
)
