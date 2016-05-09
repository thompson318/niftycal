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

set(Eigen_FOUND 0)

find_path(Eigen_INCLUDE_DIR
  NAMES Eigen/Eigen
  PATHS ${Eigen_DIR} ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES include include/eigen3
)

if(Eigen_INCLUDE_DIR)
  set(Eigen_FOUND 1)
endif()

