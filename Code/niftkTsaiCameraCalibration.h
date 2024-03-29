/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTsaiCameraCalibration_h
#define niftkTsaiCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include <cv.h>

namespace niftk
{

/**
* \file niftkTsaiCameraCalibration.h
* \brief Mono and Stereo calibration routines using Tsai 1987 method.
* \ingroup calibration
*/

/**
* \brief Performs a mono camera calibration using Tsai 1987 method.
* If ITK is not compiled in, you just have the initial linear bit, which
* can be a bit poor, so for Tsai, you are advised to use ITK to get LM optimisation.
* This method will check for planarity and then switch to either the coplanar or the non-coplanar method.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double TsaiMonoCameraCalibration(const niftk::Model3D& model3D,
                                                    const niftk::PointSet& imagePoints2D,
                                                    const cv::Size2i& imageSize,
                                                    const cv::Point2d& sensorDimensions,
                                                    cv::Mat& intrinsic3x3,
                                                    cv::Mat& distortion1x5,
                                                    cv::Mat& rvec1x3,
                                                    cv::Mat& tvec1x3,
                                                    const bool& fullOptimisation = true
                                                   );

/**
* \brief Performs a stereo camera calibration using Tsai 1987 method.
* If ITK is not compiled in, you just have the initial linear bit, which
* can be a bit poor, so for Tsai, you are advised to use ITK to get LM optimisation.
* This method will check for planarity and then switch to either the coplanar or the non-coplanar method.
* This method assumes the left and right camera have already been optimised, and then optimises
* the extrinsic (2 x 6 DOF) parameters using the combined left and right re-projection error.
* \param cvFlags See OpenCV docs, e.g. CV_CALIB_USE_INTRINSIC_GUESS, CV_CALIB_FIX_INTRINSIC etc.
* \param optimise3D if true will use Levenberg-Marquart to optimise the 3D reconstruction error (not recommended).
* \see niftk::StereoCameraCalibration
* \return rms re-projection and 3D reconstruction error
*/
NIFTYCAL_WINEXPORT cv::Matx21d TsaiStereoCameraCalibration(const niftk::Model3D& model3D,
                                                           const niftk::PointSet& points2DLeft,
                                                           const niftk::PointSet& points2DRight,
                                                           const cv::Size2i& imageSize,
                                                           cv::Mat& intrinsic3x3Left,
                                                           cv::Mat& distortion1x5Left,
                                                           cv::Mat& rvec1x3Left,
                                                           cv::Mat& tvec1x3Left,
                                                           cv::Mat& intrinsic3x3Right,
                                                           cv::Mat& distortion1x5Right,
                                                           cv::Mat& rvec1x3Right,
                                                           cv::Mat& tvec1x3Right,
                                                           cv::Mat& leftToRightRotationMatrix3x3,
                                                           cv::Mat& leftToRightTranslationVector3x1,
                                                           cv::Mat& essentialMatrix,
                                                           cv::Mat& fundamentalMatrix,
                                                           const int& cvFlags = 0,
                                                           const bool& optimise3D = false
                                                          );

} // end namespace

#endif
