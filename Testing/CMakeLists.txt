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
  niftkMatrixAverageTest
  niftkWarpChessboardByHomographyTest
  niftkInterpolateQuadraticSurfaceTest
  niftkExtractChessboardPointsTest
  niftkExtractCirclesPointsTest
  niftkExtractRingsPointsTest
  niftkMonoChessboardCameraCalibrationTest
  niftkIterativeMonoChessboardCameraCalibrationTest
  niftkStereoChessboardCameraCalibrationTest
  niftkIterativeStereoChessboardCameraCalibrationTest
  niftkIterativeStereoCirclesCameraCalibrationTest
  niftkIterativeStereoAprilTagsCameraCalibrationTest
  niftkIterativeMonoAprilTagsCameraCalibrationTest
  niftkMonoChessboardHandEyeCalibrationTest
  niftkIterativeMonoRingsCameraCalibrationTest
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
add_test(InterpSurf1 niftkInterpolateQuadraticSurfaceTest)
add_test(PointDist niftkPointDistortionTest ${DATA_DIR}/Laparoscope/calib.left.intrinsic.txt ${DATA_DIR}/Laparoscope/calib.left.distortion.txt 1920 1080 1098.3 267.087)
add_test(WarpHomog1 niftkWarpChessboardByHomographyTest ${DATA_DIR}/Laparoscope/Left/left-1095.png ${DATA_DIR}/Laparoscope/Left/left-1338.png 14 10 1.0880866642) #RMS error of difference in points.
add_test(AveMatrix1 niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4)
add_test(AveMatrix2 niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat3.4x4)
add_test(AveMatrix3 niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat3.4x4)
add_test(ExtChessSim1 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png 1 1 1500 1100 14 10)
add_test(ExtChessSim2 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_750_550_14_10.png 1 1 750 550 14 10)
add_test(ExtChessSim3 niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_375_275_14_10.png 1 1 375 275 14 10)
add_test(ExtChessZhang1 niftkExtractChessboardPointsTest ${DATA_DIR}/OpenCV-2.4.11/left01.png 1 1 640 480 9 6)
add_test(ExtChessCMU1 niftkExtractChessboardPointsTest ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp 1 1 720 480 10 7)
add_test(ExtCircles1 niftkExtractCirclesPointsTest ${DATA_DIR}/SimulationDots1/Left1.png 1280 1024 11 4)
add_test(ExtAprilTags1 niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 36h11 1 1 792 612 35)
add_test(ExtAprilTags2 niftkExtractAprilTagsPointsTest ${DATA_DIR}/SimulationAprilTags/Left1.png 36h11 1 1 1280 1024 35)
add_test(ExtAprilTags3 niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/apriltags-25h7-9x8.png 25h7 1 1 7632 6784 72)
add_test(ExtRings1 niftkExtractRingsPointsTest ${DATA_DIR}/CMU/ring_12_9.bmp ${DATA_DIR}/CMU/ring_12_9.bmp ${DATA_DIR}/CMU/ring_10_7_on_12_9.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp 8496 6372 10 7 ${DATA_DIR}/CMU/ring_10_7_on_12_9.txt 260000 1 0.1)
add_test(ExtRings2 niftkExtractRingsPointsTest ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_-15_0_-5_.bmp ${DATA_DIR}/CMU/ring_12_9.bmp ${DATA_DIR}/CMU/ring_10_7_on_12_9.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp 720 480 10 7 ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_-15_0_-5_test.txt 10000 0 2.0)

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
  -0.0198931 0.0402461 0.000409826               # rvec
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
  ${DATA_DIR}/Simulation2/Left1.png
  ${DATA_DIR}/Simulation2/Left2.png
  ${DATA_DIR}/Simulation2/Left3.png
  ${DATA_DIR}/Simulation2/Left4.png
  ${DATA_DIR}/Simulation2/Left5.png
  ${DATA_DIR}/Simulation2/Left6.png
  ${DATA_DIR}/Simulation2/Left7.png
  ${DATA_DIR}/Simulation2/Left8.png
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
  ${DATA_DIR}/Simulation2/Left1.png
  ${DATA_DIR}/Simulation2/Left2.png
  ${DATA_DIR}/Simulation2/Left3.png
  ${DATA_DIR}/Simulation2/Left4.png
  ${DATA_DIR}/Simulation2/Left5.png
  ${DATA_DIR}/Simulation2/Left6.png
  ${DATA_DIR}/Simulation2/Left7.png
  ${DATA_DIR}/Simulation2/Left8.png
)

# Now try iterative on stereo, using simulation 2.
add_test(CalibIterStereoSim1 niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model points
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.txt # 2D points for reference image
  1                                                     # zero distortion (1==true)
  -3.36426e-05 -0.0856378 -8.14416e-05 # rvec
  5.98069 0.00132401 0.519151 # tvec
  ${DATA_DIR}/Simulation2/Left1.png
  ${DATA_DIR}/Simulation2/Left2.png
  ${DATA_DIR}/Simulation2/Left3.png
  ${DATA_DIR}/Simulation2/Left4.png
  ${DATA_DIR}/Simulation2/Left5.png
  ${DATA_DIR}/Simulation2/Left6.png
  ${DATA_DIR}/Simulation2/Left7.png
  ${DATA_DIR}/Simulation2/Left8.png
  ${DATA_DIR}/Simulation2/Right1.png
  ${DATA_DIR}/Simulation2/Right2.png
  ${DATA_DIR}/Simulation2/Right3.png
  ${DATA_DIR}/Simulation2/Right4.png
  ${DATA_DIR}/Simulation2/Right5.png
  ${DATA_DIR}/Simulation2/Right6.png
  ${DATA_DIR}/Simulation2/Right7.png
  ${DATA_DIR}/Simulation2/Right8.png
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
# the same reference image produces too high distortion on one channel,
# causing subsequent circle extraction to fail.
#add_test(CalibIterStereoCircles niftkIterativeStereoCirclesCameraCalibrationTest
#  ${DATA_DIR}/SimulationDots1/Left1.png                     # Model image
#  ${DATA_DIR}/SimulationDots1/circles_11_4_3.txt            # Model description (grid points 3D)
#  11 4                                                      # pattern size
#  ${DATA_DIR}/SimulationDots1/reference_1280_1024_11_4.txt  # 2D points for reference image
#  1                                                         # zero distortion (1==true)
#  0.995869 0.00066861 -0.0907979 # rvec
#  2.43657 -0.000460828 0.262795 # tvec
#  ${DATA_DIR}/SimulationDots1/Left1.png
#  ${DATA_DIR}/SimulationDots1/Left2.png
#  ${DATA_DIR}/SimulationDots1/Left3.png
#  ${DATA_DIR}/SimulationDots1/Left4.png
#  ${DATA_DIR}/SimulationDots1/Left5.png
#  ${DATA_DIR}/SimulationDots1/Left6.png
#  ${DATA_DIR}/SimulationDots1/Left7.png
#  ${DATA_DIR}/SimulationDots1/Left8.png
#  ${DATA_DIR}/SimulationDots1/Left9.png
#  ${DATA_DIR}/SimulationDots1/Right1.png
#  ${DATA_DIR}/SimulationDots1/Right2.png
#  ${DATA_DIR}/SimulationDots1/Right3.png
#  ${DATA_DIR}/SimulationDots1/Right4.png
#  ${DATA_DIR}/SimulationDots1/Right5.png
#  ${DATA_DIR}/SimulationDots1/Right6.png
#  ${DATA_DIR}/SimulationDots1/Right7.png
#  ${DATA_DIR}/SimulationDots1/Right8.png
#  ${DATA_DIR}/SimulationDots1/Right9.png
#)

# Now try iterative, mono, using CMU synthetic dataset.
add_test(CalibIterMonoCMU1 niftkIterativeMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/CMU/TestSyntheticData/checker1/45_rz0/0/checker_0_0_0_.bmp # Model image
  ${DATA_DIR}/CMU/TestSyntheticData/chessboard_10_7_3.txt                # Model description (chessboard points)
  10 7                                                                   # internal corners
  ${DATA_DIR}/CMU/TestSyntheticData/reference_720_480_10_7.txt           # 2D points for reference image
  0                                                                      # zero distortion (1==true)
  0.121773                                                               # RMS
  1147.76 1148.09 361.835 239.6                                          # intrinsic. Should be 1150, 1150, 360.5 240.5.
  -0.737653 1.13968 0.000389667 -0.000417089                             # distortion. Should be -0.7, -0.3, 0.004, 0.00005, 8.
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
add_test(CalibIterStereoAprilSim niftkIterativeStereoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  1                                                         # zero distortion (1==true)
  1 1                                                       # Simulation has proper scale factors
  2.77247e-05 -0.0853894 2.02478e-05 # rvec
  5.97596 0.000262982 0.519377 # tvec
  ${DATA_DIR}/SimulationAprilTags/Left1.png
  ${DATA_DIR}/SimulationAprilTags/Left2.png
  ${DATA_DIR}/SimulationAprilTags/Left3.png
  ${DATA_DIR}/SimulationAprilTags/Left4.png
  ${DATA_DIR}/SimulationAprilTags/Left5.png
  ${DATA_DIR}/SimulationAprilTags/Left6.png
  ${DATA_DIR}/SimulationAprilTags/Left7.png
  ${DATA_DIR}/SimulationAprilTags/Left8.png
  ${DATA_DIR}/SimulationAprilTags/Left9.png
  ${DATA_DIR}/SimulationAprilTags/Right1.png
  ${DATA_DIR}/SimulationAprilTags/Right2.png
  ${DATA_DIR}/SimulationAprilTags/Right3.png
  ${DATA_DIR}/SimulationAprilTags/Right4.png
  ${DATA_DIR}/SimulationAprilTags/Right5.png
  ${DATA_DIR}/SimulationAprilTags/Right6.png
  ${DATA_DIR}/SimulationAprilTags/Right7.png
  ${DATA_DIR}/SimulationAprilTags/Right8.png
  ${DATA_DIR}/SimulationAprilTags/Right9.png
)

# Now try iterative AprilTags on real data.
add_test(CalibIterStereoAprilReal1 niftkIterativeStereoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  0                                                         # zero distortion (1==true)
  1 2                                                       # Simulation has proper scale factors
  0.00367244 -0.03346 -0.00269221 # rvec
  -2.73359 0.0348689 -0.0495921 # tvec
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230251914006800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230251962009600_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252058015000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252090016800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252154020400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252458038000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230253498097400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230256554272200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257324316200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257658335200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257722339000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230260762512800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230271146106800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230274842318200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230280181623600_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230284714882800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296106534400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296330547200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296794573800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230320793946400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230251914006800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230251962009600_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230252058015000_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230252090016800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230252154020400_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230252458038000_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230253498097400_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230256554272200_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230257324316200_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230257658335200_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230257722339000_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230260762512800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230271146106800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230274842318200_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230280181623600_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230284714882800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230296106534400_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230296330547200_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230296794573800_right.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/right/1458230320793946400_right.bmp
)

add_test(CalibIterMonoAprilReal1 niftkIterativeMonoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  0                                                         # zero distortion (1==true)
  1 2                                                       # Simulation has proper scale factors
  0.642218                                                  # RMS
  1747.17 870.803 894.99 236.136                            # intrinsic
  -0.331478 0.316505 0.00741399 -0.00205115                 # distortion
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230251914006800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230251962009600_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252058015000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252090016800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252154020400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230252458038000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230253498097400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230256554272200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257324316200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257658335200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230257722339000_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230260762512800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230271146106800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230274842318200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230280181623600_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230284714882800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296106534400_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296330547200_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230296794573800_left.bmp
  ${DATA_DIR}/AprilTagsMono-2016.03.17/left/1458230320793946400_left.bmp
)

add_test(CalibHandEyeMono niftkMonoChessboardHandEyeCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/calib.left.handeye.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame1.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame2.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame3.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame4.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame5.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame6.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame7.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame8.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame9.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame10.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame11.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame12.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame13.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame14.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame15.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame16.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame17.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame18.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame19.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame20.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame21.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame22.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame23.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame24.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame25.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame26.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame27.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame28.png
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame29.png
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame30.png
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame31.png
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame32.png
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/LeftFrame33.png
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/0.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/1.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/2.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/3.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/4.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/5.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/6.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/7.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/8.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/9.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/10.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/11.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/12.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/13.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/14.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/15.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/16.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/17.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/18.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/19.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/20.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/21.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/22.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/23.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/24.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/25.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/26.txt
  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/27.txt
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/28.txt
#   ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/29.txt
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/30.txt
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/31.txt
#  ${DATA_DIR}/2015.06.04_14-34-29-279/polaris.1/TrackerMatrices1/32.txt
)

add_test(CalibIterMonoRingsSim1 niftkIterativeMonoRingsCameraCalibrationTest
  ${DATA_DIR}/CMU/ring_12_9.bmp                         # Model image
  ${DATA_DIR}/CMU/rings_10_7_3.txt                      # Model points
  ${DATA_DIR}/CMU/ring_10_7_on_12_9.txt                 # 2D points for reference image
  ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp
  10 7                                                  # number rings
  0                                                     # zero distortion (1==true)
  0.0127745                                             # RMS
  1150.06 1150.06 359.523 239.518                       # intrinsic (should be 1750, 1750, 639.5, 511.5)
  -0.719492 0.228066 0.000388386 3.21593e-05            # distortion (should be 0, 0, 0, 0)
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_-15_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_-15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_15_0_-5_.bmp
)
