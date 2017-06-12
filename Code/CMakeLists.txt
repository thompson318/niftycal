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
  Internal/niftkTsaiUtilities.cxx
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
  niftkRenderingCameraCalibration.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS Detectors/niftkAprilTagsPointDetector.cxx)
endif()

if(ITK_FOUND)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNoIntrinsicsCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMonoCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsaiCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMaltiHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNDOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNDOFHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai3ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai3ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai2ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai2ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai5ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai5ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai8ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai8ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai10ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai10ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai11ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai11ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereo2DCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEye2DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEye2DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCameraCalibration2DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCameraCalibration2DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereo3DCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoExtrinsicsCalibration3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoExtrinsicsCalibration3DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoIntrinsicsCalibration3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoIntrinsicsCalibration3DOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEye3DCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEye3DOptimiser.cxx)

endif()

if(VTK_FOUND)
  list(APPEND NIFTYCAL_SRCS Rendering/niftkCalibratedCamera.cxx)
  list(APPEND NIFTYCAL_SRCS Rendering/niftkCalibratedRenderingPipeline.cxx)
endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

add_subdirectory(Apps)
