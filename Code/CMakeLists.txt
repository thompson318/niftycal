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
  niftkMaxERE.cxx
  niftkIPoint2DDetector.cxx
  niftkTemplateMatching.cxx
  niftkMonoCameraCalibration.cxx
  niftkStereoCameraCalibration.cxx
  niftkIterativeMonoCameraCalibration.cxx
  niftkIterativeStereoCameraCalibration.cxx
  niftkTimingLagCalibration.cxx
  niftkHandEyeCalibration.cxx
  niftkTsaiCameraCalibration.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS Detectors/niftkAprilTagsPointDetector.cxx)
endif()

if(ITK_FOUND)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearHandEyeCostFunction.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearMaltiHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearNoIntrinsicsHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinear12DOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinear12DOFHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearNDOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearNDOFHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoHandEyeOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoCalibrationOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoExtrinsicsCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoExtrinsicsCalibrationOptimiser.cxx)

  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoIntrinsicsCalibrationCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS Optimisers/niftkNonLinearStereoIntrinsicsCalibrationOptimiser.cxx)

endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

add_subdirectory(Apps)
