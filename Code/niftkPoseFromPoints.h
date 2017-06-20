/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkPoseFromPoints_h
#define niftkPoseFromPoints_h

#include "niftkWin32ExportHeader.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <cv.h>

namespace niftk
{

/**
* \file niftkPoseFromPoints.h
* \brief Estimates the camera pose given a camera calibration and the image and model points
*/
NIFTYCAL_WINEXPORT void PoseFromPoints(const Model3D& model,
                                       const std::list<PointSet>& listOfPointSets,
                                       cv::Mat& intrinsic,
                                       cv::Mat& distortion,
                                       std::vector<cv::Mat>& rvecs,
                                       std::vector<cv::Mat>& tvecs
                                      );

} // end namespace

#endif
