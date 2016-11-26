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
  Detectors/niftkRingsPointDetector.cxx
  Detectors/niftkCirclesIterativePointDetector.cxx
  niftkNiftyCalException.cxx
  niftkNiftyCalTypes.cxx
  niftkPointUtilities.cxx
  niftkHomographyUtilities.cxx
  niftkMatrixUtilities.cxx
  niftkIOUtilities.cxx
  niftkIPoint2DDetector.cxx
  niftkTemplateMatching.cxx
  niftkMonoCameraCalibration.cxx
  niftkStereoCameraCalibration.cxx
  niftkIterativeMonoCameraCalibration.cxx
  niftkIterativeStereoCameraCalibration.cxx
  niftkHandEyeCalibration.cxx
  niftkTsaiCameraCalibration.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS Detectors/niftkAprilTagsPointDetector.cxx)
endif()

if(ITK_FOUND)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearHandEyeCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearMaltiHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNoIntrinsicsHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinear12DOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinear12DOFHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNDOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearNDOFHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoCalibrationOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoExtrinsicsCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoExtrinsicsCalibrationOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoIntrinsicsCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearStereoIntrinsicsCalibrationOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsaiCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsaiNormalisingCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai3ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai3ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai5ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai5ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai8ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai8ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai10ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai10ParamCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai11ParamOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Internal/niftkNonLinearTsai11ParamCostFunction.cxx)

endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

add_subdirectory(Apps)
