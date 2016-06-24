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
  niftkNiftyCalException.cxx
  niftkNiftyCalTypes.cxx
  niftkPointUtilities.cxx
  niftkHomographyUtilities.cxx
  niftkMatrixUtilities.cxx
  niftkIOUtilities.cxx
  niftkTemplateMatching.cxx
  niftkIPoint2DDetector.cxx
  niftkPointDetector.cxx
  niftkChessboardPointDetector.cxx
  niftkCirclesPointDetector.cxx
  niftkTemplateMatchingPointDetector.cxx
  niftkRingsPointDetector.cxx
  niftkCirclesIterativePointDetector.cxx
  niftkMonoCameraCalibration.cxx
  niftkStereoCameraCalibration.cxx
  niftkIterativeMonoCameraCalibration.cxx
  niftkIterativeStereoCameraCalibration.cxx
  niftkMaxERE.cxx
  niftkCalibrateTimingLag.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS niftkAprilTagsPointDetector.cxx)
endif()

if(ITK_FOUND)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMalti12DOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMalti12DOFHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiNDOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiNDOFHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiStereoHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiStereoHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearStereoCalibrationOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearStereoCalibrationCostFunction.cxx)
endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

add_subdirectory(Apps)
