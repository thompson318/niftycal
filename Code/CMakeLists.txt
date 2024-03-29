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

set(NIFTYCAL_SRCS
  Internal/niftkIterativeCalibrationUtilities.cxx
  Internal/niftkTriangulationUtilities.cxx
  Internal/niftkCalibrationUtilities.cxx
  Internal/Tsai/niftkTsaiUtilities.cxx
  Detectors/niftkPointDetector.cxx
  Detectors/niftkChessboardPointDetector.cxx
  Detectors/niftkCirclesPointDetector.cxx
  Detectors/niftkTemplateMatchingPointDetector.cxx
  Detectors/niftkTemplateRingsPointDetector.cxx
  Detectors/niftkTemplateCirclesPointDetector.cxx
  Detectors/niftkSideBySideDetector.cxx
  Detectors/niftkBallDetector.cxx
  Detectors/niftkRedBallDetector.cxx
  Detectors/niftkWhiteBallDetector.cxx
  niftkNiftyCalException.cxx
  niftkNiftyCalTypes.cxx
  niftkPointUtilities.cxx
  niftkHomographyUtilities.cxx
  niftkMatrixUtilities.cxx
  niftkIOUtilities.cxx
  niftkIPoint2DDetector.cxx
  niftkTemplateMatching.cxx
  niftkZhangCameraCalibration.cxx
  niftkTsaiCameraCalibration.cxx
  niftkStereoCameraCalibration.cxx
  niftkHandEyeCalibration.cxx
  niftkIterativeMonoCameraCalibration.cxx
  niftkIterativeStereoCameraCalibration.cxx
  niftkTimingCalibration.cxx
  niftkPoseFromPoints.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS Detectors/niftkAprilTagsPointDetector.cxx)
endif()

if(ITK_FOUND)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNoIntrinsicsCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMonoCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereo2DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereo3DCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsaiCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai3ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai3ParamCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai2ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai2ParamCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai5ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai5ParamCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai8ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai8ParamCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai10ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai10ParamCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai11ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Tsai/niftkNonLinearTsai11ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/Stereo/niftkNonLinearStereoCameraCalibration2DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Stereo/niftkNonLinearStereoCameraCalibration2DOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Stereo/niftkNonLinearStereoExtrinsicsCalibration3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/Stereo/niftkNonLinearStereoExtrinsicsCalibration3DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearMaltiHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearNDOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearNDOFHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearStereoHandEye2DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearStereoHandEye2DOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearStereoHandEye3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/HandEye/niftkNonLinearStereoHandEye3DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/NotUsed/niftkNonLinearStereoIntrinsicsCalibration3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/NotUsed/niftkNonLinearStereoIntrinsicsCalibration3DOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/NotUsed/niftkNonLinearMonoCostFunction3D.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/NotUsed/niftkNonLinearMonoCameraCalibration3DOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/NotUsed/niftkNonLinearMonoCameraCalibration3DCostFunction.cxx)

endif()


###############################################################################
# Start of Highly Experimental Section: Do not use.
###############################################################################
#list(APPEND CMAKE_PREFIX_PATH /usr/local/lib/cmake/Ceres/)
#list(APPEND CMAKE_PREFIX_PATH /Users/mattclarkson/build/CMakeCatchTemplate-SuperBuild-Qt5/gflags/install)
#list(APPEND CMAKE_PREFIX_PATH /Users/mattclarkson/build/CMakeCatchTemplate-SuperBuild-Qt5/glog/install)
#
#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})
#
#list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearCeresStereoOptimiser.cxx)
#list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearCeresMonoOptimiser.cxx)
#
#include_directories(/Users/mattclarkson/install/gsl-2.5/include)
###############################################################################
# End of Highly Experimental Section: Do not use.
###############################################################################

###############################################################################
# Experimental: Try to calibrate based on rendering the model.
# Conclusion  : Slow, and not much benefit at the moment.
###############################################################################
if(VTK_FOUND AND ITK_FOUND AND QT_FOUND)
  list(APPEND NIFTYCAL_SRCS IntensityBased/niftkCalibratedCamera.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/niftkCalibratedRenderingPipeline.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/niftkIntensityBasedCameraCalibration.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkIntensityBasedCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkRenderingBasedCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkRenderingBasedMonoIntrinsicCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkRenderingBasedMonoExtrinsicCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkRenderingBasedMonoBlurringCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkRenderingBasedStereoExtrinsicCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkProjectionBasedCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkProjectionBasedMonoIntrinsicCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkProjectionBasedMonoExtrinsicCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS IntensityBased/Internal/niftkProjectionBasedStereoExtrinsicCostFunction.cxx)
endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${CERES_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})

add_subdirectory(Apps)
