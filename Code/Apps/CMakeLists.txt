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
  niftkStereoIterativeAprilTagsCalibration
  niftkStereoIterativeCirclesCalibration
  niftkStereoIterativeRingsCalibration
  niftkStereoChessboardCalibration
  niftkTimingCalibrationFromPoints
  niftkPoseFromPoints
)
find_package(Qt4)
if (QT_FOUND)
  include(${QT_USE_FILE})
  list(APPEND ${ALL_THIRD_PARTY_LIBRARIES} ${QT_LIBRARIES})
endif()

if (VTK_FOUND AND QT_FOUND)
  list(APPEND NIFTYCAL_APPS niftkRenderCalibratedModel)
endif()

foreach(app ${NIFTYCAL_APPS})
  add_executable(${app} ${app}.cxx)
  target_link_libraries(${app} niftycal ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()
