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
#include "niftkNiftyCalTypes.h"
#include <cv.h>
#include <list>

namespace niftk
{

/**
* \file niftkStereoCameraCalibration.h
* \brief Stereo calibration routine.
* \ingroup calibration
*/

/**
* \brief Performs a stereo camera calibration.
* \param cvFlags See OpenCV docs, e.g. CV_CALIB_USE_INTRINSIC_GUESS, CV_CALIB_FIX_INTRINSIC etc.
* \param optimise3D if true will use Levenberg-Marquart to optimise the 3D reconstruction error (not recommended).
* \return rms re-projection and 3D reconstruction error
* \throw Requires that listOfLeftHandPointSets.size() == listOfRightHandPointSets.size(),
* and that each corresponding pair of point set has at least 4 corresponding points, meaning
* the same point ID is visible in both left and right view.
*
* This method, ONLY does the stereo calibration. Also, OpenCV notes that this
* can be unstable, so you are advised to calibrate both left and right cameras
* separately first, and then call this method with cvFlags = CV_CALIB_USE_INTRINSIC_GUESS
* or CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC (better).
*/
NIFTYCAL_WINEXPORT cv::Matx21d StereoCameraCalibration(const Model3D& model,
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
                                                       cv::Mat& leftToRightRotationMatrix,
                                                       cv::Mat& leftToRightTranslationVector,
                                                       cv::Mat& essentialMatrix,
                                                       cv::Mat& fundamentalMatrix,
                                                       const int& cvFlags,
                                                       const bool& optimise3D
                                                      );

/**
* \brief As above, but does mono and stereo, Tsai 1987 or Zhang 2000.
*/
NIFTYCAL_WINEXPORT cv::Matx21d FullStereoCameraCalibration(const Model3D& model,
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
                                                           cv::Mat& leftToRightRotationMatrix,
                                                           cv::Mat& leftToRightTranslationVector,
                                                           cv::Mat& essentialMatrix,
                                                           cv::Mat& fundamentalMatrix,
                                                           const int& cvFlags,
                                                           const bool& optimise3D
                                                          );

} // end namespace

#endif
