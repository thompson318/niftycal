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
  niftkNiftyCalException.cxx
  niftkNiftyCalTypes.cxx
  niftkPointUtilities.cxx
  niftkHomographyUtilities.cxx
  niftkMatrixUtilities.cxx
  niftkIOUtilities.cxx
  niftkIterativeCalibrationUtilities.cxx
  niftkIPoint2DDetector.cxx
  niftkOpenCVPointDetector.cxx
  niftkOpenCVChessboardPointDetector.cxx
  niftkOpenCVCirclesPointDetector.cxx
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
  list(APPEND NIFTYCAL_SRCS niftkNonLinearHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMaltiHandEyeOptimiser.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMalti12DOFHandEyeCostFunction.cxx)
  list(APPEND NIFTYCAL_SRCS niftkNonLinearMalti12DOFHandEyeOptimiser.cxx)
endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})

add_executable(niftkMonoCalibrationFromPoints niftkMonoCalibrationFromPoints.cxx)
target_link_libraries(niftkMonoCalibrationFromPoints niftycal ${ALL_THIRD_PARTY_LIBRARIES})
add_executable(niftkStereoCalibrationFromPoints niftkStereoCalibrationFromPoints.cxx)
target_link_libraries(niftkStereoCalibrationFromPoints niftycal ${ALL_THIRD_PARTY_LIBRARIES})
