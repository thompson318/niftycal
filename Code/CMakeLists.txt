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
  niftkIOptimiser.cxx
  niftkIPoint2DDetector.cxx
  niftkOpenCVChessboardPointDetector.cxx
  niftkModel3D.cxx
  niftkMonoCameraCalibration.cxx
  niftkStereoCameraCalibration.cxx
)

if(AprilTags_FOUND AND Eigen_FOUND)
  list(APPEND NIFTYCAL_SRCS niftkAprilTagsPointDetector.cxx)
endif()

add_library(niftycal ${NIFTYCAL_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DNIFTYCAL_WINDOWS_EXPORT=1)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(niftycal
  PRIVATE ${OpenCV_LIBS} ${AprilTags_LIBRARIES}
)
