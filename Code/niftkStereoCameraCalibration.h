/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkStereoCameraCalibration_h
#define niftkStereoCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <cv.h>

namespace niftk
{

/**
* \file niftkStereoCameraCalibration.h
* \brief Performs a stereo camera calibration using the standard OpenCV approach.
* \param cvFlags See OpenCV docs, e.g. CV_CALIB_USE_INTRINSIC_GUESS, CV_CALIB_FIX_INTRINSIC etc.
* \return rms re-projection error over both views
*
* \throw Requires that listOfLeftHandPointSets.size() == listOfRightHandPointSets.size(),
* and that each corresponding point set has at least 1 corresponding points, meaning
* the same point ID is visible in both left and right view.
*
* This method, ONLY does the stereo calibration. Also, OpenCV notes that this
* can be unstable, so you are advised to calibrate both left and right cameras
* separately first, and then call this method with CV_CALIB_USE_INTRINSIC_GUESS
* or even CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC. etc.
*/
NIFTYCAL_WINEXPORT double StereoCameraCalibration(const Model3D& model,
                                                  const std::list<PointSet>& listOfLeftHandPointSets,
                                                  const std::list<PointSet>& listOfRightHandPointSets,
                                                  const cv::Size2i& imageSize,
                                                  cv::Mat& intrinsicLeft,
                                                  cv::Mat& distortionLeft,
                                                  std::vector<cv::Mat>& rvecsLeft,
                                                  std::vector<cv::Mat>& tvecsLeft,
                                                  cv::Mat& intrinsicRight,
                                                  cv::Mat& distortionRight,
                                                  std::vector<cv::Mat>& rvecsRight,
                                                  std::vector<cv::Mat>& tvecsRight,
                                                  cv::Mat& rightToLeftRotationMatrix,
                                                  cv::Mat& rightToLeftTranslationVector,
                                                  cv::Mat& essentialMatrix,
                                                  cv::Mat& fundamentalMatrix,
                                                  const int& cvFlags = 0
                                                 );

} // end namespace

#endif
