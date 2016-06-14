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
  niftkMonoCalibrationFromPoints
  niftkStereoCalibrationFromPoints
  niftkStereoSimulationFromPoints
  niftkStereoIterativeAprilTagsCalibration
  niftkStereoIterativeRingsCalibration
  niftkStereoChessboardCalibration
)
foreach(app ${NIFTYCAL_APPS})
  add_executable(${app} ${app}.cxx)
  target_link_libraries(${app} niftycal ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()
