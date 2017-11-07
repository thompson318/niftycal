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
set(NIFTYCAL_APPS
  niftkStereoSimulationWithNoise
  niftkStereoSimulationByParameter
  niftkMonoCalibrationFromPoints
  niftkStereoCalibrationFromPoints
  niftkStereoIterativeCirclesCalibration
  niftkStereoIterativeRingsCalibration
  niftkStereoChessboardCalibration
  niftkTimingCalibrationFromPoints
  niftkPoseFromPoints
  niftkHandEyeCalibrationUsingPointToLine
)

if (VTK_FOUND AND QT_FOUND)
  list(APPEND NIFTYCAL_APPS niftkRenderCalibratedModel)
  if(ITK_FOUND)
#    list(APPEND NIFTYCAL_APPS niftkAprilTagsCalibrationWithRendering)
  endif()
endif()

if (AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_APPS niftkStereoIterativeAprilTagsCalibration)
endif()

foreach(app ${NIFTYCAL_APPS})
  add_executable(${app} ${app}.cxx)
  target_link_libraries(${app} niftycal ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()
