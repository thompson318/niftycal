/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkZhangCameraCalibration_h
#define niftkZhangCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include <cv.h>
#include <list>

namespace niftk
{

/**
* \file niftkZhangCameraCalibration.h
* \brief Mono calibration routine using Zhang 2000 method (standard OpenCV approach).
* \ingroup calibration
*/

/**
* \brief Performs a mono camera calibration using the standard Zhang 2000 / OpenCV approach.
* \param cvFlags See OpenCV docs, e.g. CV_CALIB_USE_INTRINSIC_GUESS, CV_CALIB_FIX_INTRINSIC etc.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double ZhangMonoCameraCalibration(const Model3D& model,
                                                     const std::list<PointSet>& listOfPointSets,
                                                     const cv::Size2i& imageSize,
                                                     cv::Mat& intrinsic,
                                                     cv::Mat& distortion,
                                                     std::vector<cv::Mat>& rvecs,
                                                     std::vector<cv::Mat>& tvecs,
                                                     const int& cvFlags = 0
                                                    );

} // end namespace

#endif
