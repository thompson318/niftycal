/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkMonoCameraCalibration_h
#define niftkMonoCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkModel3D.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <cv.h>

namespace niftk
{

/**
* \file niftkMonoCameraCalibration.h
* \brief Performs a mono camera calibration.
* \return rms error of re-projected 3D points onto 2D points
*/
NIFTYCAL_WINEXPORT double MonoCameraCalibration(const Model3D& model,
                                                const std::list<PointSet>& listOfPointSets,
                                                const cv::Size2i& imageSize,
                                                cv::Mat& intrinsic,
                                                cv::Mat& distortion,
                                                std::vector<cv::Mat>& rvecs,
                                                std::vector<cv::Mat>& tvecs,
                                                const bool& intrinsicsFixed = false
                                               );

} // end namespace

#endif

