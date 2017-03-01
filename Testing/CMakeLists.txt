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
set(TMP_DIR ${CMAKE_BINARY_DIR}/Testing/Temporary)

set(TEST_CASES
  niftkNiftyCalExceptionTest
  niftkPointUtilitiesTest
  niftkPointDistortionTest
  niftkMatrixAverageTest
  niftkStereoMatrixIOTest
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
  niftkIterativeTemplateMatchingCameraCalibrationTest
  niftkTsaiMonoCalibrationTest
  niftkTsaiStereoCalibrationTest
  niftkTsaiReferenceNonCoplanerCalibrationTest
  niftkTimingCalibrationUtilitiesTest
  niftkTimingCalibrationTest
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

# Put unit-tests at the top. There is no point trying to assess the correctness of functional
# requirements, if the internal and most lowest level unit tests themselves do not work.
add_test(UNIT-NiftyCalExcept ${EXECUTABLE_OUTPUT_PATH}/niftkNiftyCalExceptionTest)
add_test(UNIT-PointUtils ${EXECUTABLE_OUTPUT_PATH}/niftkPointUtilitiesTest)
add_test(UNIT-PointDist ${EXECUTABLE_OUTPUT_PATH}/niftkPointDistortionTest ${DATA_DIR}/Laparoscope/calib.left.intrinsic.txt ${DATA_DIR}/Laparoscope/calib.left.distortion.txt 1920 1080 1098.3 267.087)
add_test(UNIT-InterpSurf1 ${EXECUTABLE_OUTPUT_PATH}/niftkInterpolateQuadraticSurfaceTest)
add_test(UNIT-WarpHomog1 ${EXECUTABLE_OUTPUT_PATH}/niftkWarpChessboardByHomographyTest ${DATA_DIR}/Laparoscope/Left/left-1095.png ${DATA_DIR}/Laparoscope/Left/left-1338.png 14 10 1.0880866642) #RMS error of difference in points.
add_test(UNIT-AveMatrix1 ${EXECUTABLE_OUTPUT_PATH}/niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4)
add_test(UNIT-AveMatrix2 ${EXECUTABLE_OUTPUT_PATH}/niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat3.4x4)
add_test(UNIT-AveMatrix3 ${EXECUTABLE_OUTPUT_PATH}/niftkMatrixAverageTest ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${DATA_DIR}/MatrixAverage1/mat2.4x4 ${DATA_DIR}/MatrixAverage1/mat3.4x4)
add_test(UNIT-StereoMatrixIO-1 ${EXECUTABLE_OUTPUT_PATH}/niftkStereoMatrixIOTest ${DATA_DIR}/MatrixAverage1/mat1.4x4 ${TMP_DIR}/calib.r2l.txt)
add_test(UNIT-ExtChessSim ${EXECUTABLE_OUTPUT_PATH}/niftkExtractChessboardPointsTest ${DATA_DIR}/Simulation2/reference_750_550_14_10.png 1 1 750 550 14 10 140 0)
add_test(UNIT-ExtAprilTags1 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractAprilTagsPointsTest ${DATA_DIR}/SimulationAprilTags/Left1.png 36h11 1 1 1280 1024 175 0)
add_test(UNIT-ExtAprilTags2 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 36h11 1 1 792 612 175 0)
add_test(UNIT-ExtAprilTags3 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/apriltags-25h7-9x8.png 25h7 1 1 7632 6784 360 0)
add_test(UNIT-TimingUtils ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationUtilitiesTest )
add_test(UNIT-Timing1 ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationTest ${DATA_DIR}/Timing/tracker3D.txt ${DATA_DIR}/Timing/image2D-1.txt  0)
add_test(UNIT-Timing2 ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationTest ${DATA_DIR}/Timing/tracker3D.txt ${DATA_DIR}/Timing/image2D-2.txt  3)
add_test(UNIT-Timing3 ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationTest ${DATA_DIR}/Timing/tracker3D.txt ${DATA_DIR}/Timing/image2D-3.txt -3)
add_test(UNIT-Timing4 ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationTest ${DATA_DIR}/Timing/tracker3D.txt ${DATA_DIR}/Timing/image2D-4.txt 45)
add_test(UNIT-TimingSpectraNVidiaSDI ${EXECUTABLE_OUTPUT_PATH}/niftkTimingCalibrationTest ${DATA_DIR}/Timing/spectra-nvidiasdi-cuda55-tracking.txt ${DATA_DIR}/Timing/spectra-nvidiasdi-cuda55-points.txt 53)


# Chess
add_test(REQ-2002-CSD-0001-001-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractChessboardPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 1 1 792 612 14 10   0 0)
add_test(REQ-2002-CSD-0001-002-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractChessboardPointsTest ${DATA_DIR}/OpenCV-2.4.11/left01.png                1 1 640 480  9  6  54 0)

# Circles
add_test(REQ-2002-CSD-0001-003-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractCirclesPointsTest ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png 792 612   11 4  0 0)
add_test(REQ-2002-CSD-0001-004-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractCirclesPointsTest ${DATA_DIR}/SimulationDots1/Left1.png               1280 1024 11 4 44 0)

# April
add_test(REQ-2002-CSD-0001-005-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractAprilTagsPointsTest ${DATA_DIR}/SimulationDots1/Left1.png 36h11 1 1 1280 1024 0 0)
add_test(REQ-2002-CSD-0001-006-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractAprilTagsPointsTest ${DATA_DIR}/AprilTags/apriltags-25h7-9x8-500border-smaller.png 25h7 1 1 1079 973 360 0)

# Rings
add_test(REQ-2002-CSD-0001-007-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractRingsPointsTest ${DATA_DIR}/SimulationDots1/Left1.png ${DATA_DIR}/CMU/rings_10_7.png ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp 1 1 1280 1024 10 7 0 0 10000  0 2.0)
add_test(REQ-2002-CSD-0001-008-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractRingsPointsTest ${DATA_DIR}/SimulationDots1/Left1.png ${DATA_DIR}/CMU/rings_10_7.png ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp 1 1 1280 1024 10 7 0 0 260000 1 0.1)
add_test(REQ-2002-CSD-0001-009-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractRingsPointsTest ${DATA_DIR}/CMU/PreciseCameraCalibration/GridPattern-rings.png ${DATA_DIR}/CMU/rings_10_7.png ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp 1 1 1400 1050 10 7 70 260000 1 0 0.1 ${DATA_DIR}/CMU/PreciseCameraCalibration/GridPattern-rings.txt)
add_test(REQ-2002-CSD-0001-014-01 ${EXECUTABLE_OUTPUT_PATH}/niftkExtractRingsPointsTest ${DATA_DIR}/Tsai/reference_780_540_21_7.png ${DATA_DIR}/Tsai/reference_780_540_21_7.png ${DATA_DIR}/Tsai/reference_780_540_21_7.txt ${DATA_DIR}/CMU/PreciseCameraCalibration/GridPattern-rings-template.png 1 1 780 540 21 7 147 260000 0 1 0.1 ${DATA_DIR}/Tsai/reference_780_540_21_7.txt)

# Test the performance of the underlying OpenCV mono calibration.
add_test(REQ-2002-CSD-0001-010-01 ${EXECUTABLE_OUTPUT_PATH}/niftkMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt
  14 10
  0.298669                                       # expected RMS, for regression testing purposes.
  1750 1750 639.5 511.5                          # intrinsic, specified during rendering.
  0 0 0 0                                        # distortion, specified during rendering
  1                                              # zeroDistortion==1 meaning 'yes, we expect zeroDistortion'.
  1                                              # fxTol, so we expect OpenCV to work out fx,fy accurate to 1.
  2                                              # fyTol, so we expect OpenCV to work out cx,cy accurate to 0.5.
  0.01                                           # distTol, so we expect OpenCV to work out distortion, accurate to 0.01, as there is no distortion, and we explicitly told OpenCV so.
  ${DATA_DIR}/Simulation2/Left1.png
  ${DATA_DIR}/Simulation2/Left2.png
  ${DATA_DIR}/Simulation2/Left3.png
  ${DATA_DIR}/Simulation2/Left4.png
  ${DATA_DIR}/Simulation2/Left5.png
  ${DATA_DIR}/Simulation2/Left6.png
  ${DATA_DIR}/Simulation2/Left7.png
  ${DATA_DIR}/Simulation2/Left8.png
  ${DATA_DIR}/Simulation2/Left9.png
)

# For Laparoscope dataset, gives the benchmark for standard OpenCV mono calibration for left camera,
# but there is no gold standard, so we just use this test as a regression test.
add_test(REQ-2002-CSD-0001-011-01 ${EXECUTABLE_OUTPUT_PATH}/niftkMonoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  0.577703                                       # RMS
  2007.33 2013.72 945.486 619.07                 # intrinsic
  -0.237351 -0.244029 0.00299158 -0.00141141     # distortion
  0                                              # zeroDistortion==0 meaning 'no, we expect some distortion'.
  0.01                                           # fxTol, so we expect OpenCV to work out fx,fy accurate to 0.01 as this is a regression test, and we have specified the result.
  0.01                                           # fyTol, so we expect OpenCV to work out cx,cy accurate to 0.01 as this is a regression test, and we have specified the result.
  0.01                                           # distTol, so we expect OpenCV to work out distortion, accurate to 0.01 as this is a regression test, and we have specified the result.
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

add_test(REQ-2002-CSD-0001-012-01 ${EXECUTABLE_OUTPUT_PATH}/niftkStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  0 0 0                                          # rvec
  6 0 0                                          # tvec
  1                                              # zeroDistortion==1 meaning 'yes, we expect zeroDistortion'.
  0.1                                            # rotation tolerance, so we expect OpenCV to work out r2l rotation parameters accurate to 0.1
  0.8                                            # translation tolerance, so we expect OpenCV to work out r2l translation parameters accurate to 0.8
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

add_test(REQ-2002-CSD-0001-013-01 ${EXECUTABLE_OUTPUT_PATH}/niftkStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt # Model description (chessboard points)
  14 10                                          # internal corners
  -0.0198931 0.0402461 0.000409826               # rvec
  -4.64005 0.26403 0.297868                      # tvec
  0                                              # zeroDistortion==0 meaning 'no, we expect some distortion'.
  0.01                                           # rotation tolerance, so we expect OpenCV to work out r2l rotation parameters accurate to 0.01
  0.01                                           # translation tolerance, so we expect OpenCV to work out r2l translation parameters accurate to 0.01
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
add_test(REG-CalibIterMonoSim1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoChessboardCameraCalibrationTest
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
add_test(REG-CalibIterMonoSim2 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoChessboardCameraCalibrationTest
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
add_test(REG-CalibIterStereoSim1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.png # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model points
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_1500_1100_14_10.txt # 2D points for reference image
  1                                                     # zero distortion (1==true)
  4.85642e-05 -0.0858263 -8.79148e-05 # rvec
  5.9781 0.00022072 0.514868 # tvec
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

# Check iterative does something on mono laparoscope.
add_test(REG-CalibIterLapLeft ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoChessboardCameraCalibrationTest
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
add_test(REG-CalibIterLapStereo ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeStereoChessboardCameraCalibrationTest
  ${DATA_DIR}/Simulation2/reference_750_550_14_10.png   # Model image
  ${DATA_DIR}/Laparoscope/chessboard_14_10_3.txt        # Model description (chessboard points)
  14 10                                                 # internal corners
  ${DATA_DIR}/Simulation2/reference_750_550_14_10.txt   # 2D points for reference image
  0                                                     # zero distortion (1==true)
  -0.0214663 0.0282256 -0.000394161 # rvec              # c.f. none iterative: 0.99919 -0.000809931 0.0402285
  -4.62868 0.265681 1.2335 # tvec                       # c.f. none iterative: -4.64005 0.26403 0.297868
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

# Now try iterative, mono, using CMU synthetic dataset.
add_test(REG-CalibIterMonoCMU1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoChessboardCameraCalibrationTest
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
add_test(REG-CalibIterMonoCMU2 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoChessboardCameraCalibrationTest
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

add_test(REG-CalibIterMonoAprilReal1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeMonoAprilTagsCameraCalibrationTest
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

# Now try iterative on AprilTags.
add_test(REG-CalibIterStereoAprilSim ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeStereoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  1                                                         # zero distortion (1==true)
  1 1                                                       # Simulation has proper scale factors
  -9.47177e-05 -0.0851061 4.46241e-05 # rvec
  5.98075 0.0046706 0.497847 # tvec
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
add_test(REG-CalibIterStereoAprilReal1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeStereoAprilTagsCameraCalibrationTest
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.png       # Model image
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in-3D.txt    # Model description (grid points 3D)
  ${DATA_DIR}/AprilTags/8.5x11in-tag-mosaic-1.5in.txt       # 2D points for reference image
  0                                                         # zero distortion (1==true)
  1 2                                                       # Simulation has proper scale factors
  0.00870366 -0.0224016 -0.00259507 # rvec
  -2.77744 0.0814374 -0.523483 # tvec
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

add_test(REG-CalibIterMonoCirclesSim1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeTemplateMatchingCameraCalibrationTest
  1                                                     # Method (0==rings, 1==circles)
  ${DATA_DIR}/CMU/circles_10_7.png                      # Model image
  ${DATA_DIR}/CMU/centres_10_7_7mm.txt                  # Model points
  ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt              # 2D points for reference image
  ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gcircleFilter.bmp # Template
  10 7                                                  # number rings
  0                                                     # zero distortion (1==true)
  0.0171658                                             # RMS
  1149.72 1149.72 359.545 239.527                       # intrinsic (should be 1150, 1150, 360.5, 240.5)
  -0.718669 0.204146 0.000387022 2.4678e-05             # distortion (should be -0.7, -0.3, 0.0004, 0.00005)
  ${DATA_DIR}/CMU/TestSyntheticData/circle1/15_rz-5/0/circle_-15_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/circle1/15_rz-5/0/circle_0_-15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/circle1/15_rz-5/0/circle_0_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/circle1/15_rz-5/0/circle_0_15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/circle1/15_rz-5/0/circle_15_0_-5_.bmp
)

add_test(REG-CalibIterMonoRingsSim1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeTemplateMatchingCameraCalibrationTest
  0                                                     # Method (0==rings, 1==circles)
  ${DATA_DIR}/CMU/rings_10_7.png                        # Model image
  ${DATA_DIR}/CMU/centres_10_7_7mm.txt                  # Model points
  ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt              # 2D points for reference image
  ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp # Template
  10 7                                                  # number rings
  0                                                     # zero distortion (1==true)
  0.0129552                                             # RMS
  1150.11 1150.1 359.568 239.511                        # intrinsic (should be 1150, 1150, 360.5, 240.5)
  -0.718422 0.194674 0.000395073 1.73756e-05            # distortion (should be -0.7, -0.3, 0.0004, 0.00005)
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_-15_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_-15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_0_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_0_15_-5_.bmp
  ${DATA_DIR}/CMU/TestSyntheticData/ring1/15_rz-5/0/ring_15_0_-5_.bmp
)


add_test(REG-CalibIterMonoRingsReal1 ${EXECUTABLE_OUTPUT_PATH}/niftkIterativeTemplateMatchingCameraCalibrationTest
  0                                                     # Method (0==rings, 1==circles)
  ${DATA_DIR}/CMU/rings_10_7.png                        # Model image
  ${DATA_DIR}/CMU/centres_10_7_7mm.txt                  # Model points
  ${DATA_DIR}/CMU/centres_10_7_on_12_9.txt              # 2D points for reference image
  ${DATA_DIR}/CMU/PreciseCameraCalibration/packagedCode/gringFilter.bmp # Template
  10 7                                                  # number rings
  0                                                     # zero distortion (1==true)
  0.0658565                                             # RMS
  841.847 841.146 511.589 362.415                       # intrinsic
  -0.439364 0.325313 0.0012054 -0.000253661             # distortion
  ${DATA_DIR}/CMU/TestRealImage/ring/1/img1.bmp
  ${DATA_DIR}/CMU/TestRealImage/ring/1/img2.bmp
  ${DATA_DIR}/CMU/TestRealImage/ring/1/img3.bmp
  ${DATA_DIR}/CMU/TestRealImage/ring/1/img4.bmp
  ${DATA_DIR}/CMU/TestRealImage/ring/1/img5.bmp
)


add_test(REG-CalibHandEyeMono ${EXECUTABLE_OUTPUT_PATH}/niftkMonoChessboardHandEyeCalibrationTest
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

add_test(REG-TsaiCPMonoL ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiMonoCalibrationTest
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_left.bmp
         ${DATA_DIR}/Tsai/Coplanar/model.txt
         10 7 1920 540 1 2 1676.02 1676.02 903.128 521.947 -0.219052 0.996367)

add_test(REG-TsaiCPMonoR ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiMonoCalibrationTest
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_right.bmp
         ${DATA_DIR}/Tsai/Coplanar/model.txt
         10 7 1920 540 1 2 1660.29 1660.29 979.617 528.893 -0.260747 1.42307)

add_test(REG-TsaiCPStereo2D ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiStereoCalibrationTest
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_left.bmp
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_right.bmp
         ${DATA_DIR}/Tsai/Coplanar/model.txt
         10 7 1920 540 1 2 1676.020778 1676.020778 903.1279753 521.9467757 -0.219052053 0.6767414659 0)

add_test(REG-TsaiCPStereo3D ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiStereoCalibrationTest
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_left.bmp
         ${DATA_DIR}/Tsai/Coplanar/1479224185706804480_right.bmp
         ${DATA_DIR}/Tsai/Coplanar/model.txt
         10 7 1920 540 1 2 1680.902938 1697.838245 904.3534964 518.0770595 -0.219052053 0.2584777885 1)

#
# Commented out until OpenCV bug is fixed, (duplicates from findCircles)
# or we have other test data.
#
#add_test(REG-TsaiNCPMonoL ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiMonoCalibrationTest
#         ${DATA_DIR}/Tsai/NonCoplanar/1479224227386188288_left.bmp
#         ${DATA_DIR}/Tsai/NonCoplanar/model-2.txt
#         10 7 1920 540 1 2 1507.9 1456.72 819.128 583.707 -0.0755172 9.10026)
#add_test(REG-TsaiNCPMonoR ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiMonoCalibrationTest
#         ${DATA_DIR}/Tsai/NonCoplanar/1479224227386188288_right.bmp
#         ${DATA_DIR}/Tsai/NonCoplanar/model-2.txt
#         10 7 1920 540 1 2 1512.35 1459.68 878.344 592.101 -0.0672333 9.57499)

add_test(REG-TsaiRefNCP ${EXECUTABLE_OUTPUT_PATH}/niftkTsaiReferenceNonCoplanerCalibrationTest
         ${DATA_DIR}/Tsai/NonCoplanar/data.txt
         576 576 0.02300000 0.02300000 3041.15 3040 262.302 212.689 3.07898 0.0898487
        )
