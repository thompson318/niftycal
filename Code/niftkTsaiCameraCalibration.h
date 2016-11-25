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
*/

/**
* \brief Performs a mono camera calibration using Tsai 1987 method.
* If ITK is not compiled in, you just have the initial linear bit.
* Will check for planarity and then switch to either the coplanar or the non-coplanar method.
* \param fullOptimisation defaults to false, to most closely match the paper.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double TsaiMonoCameraCalibration(const niftk::Model3D& model3D,
                                                    const niftk::PointSet& imagePoints2D,
                                                    const cv::Size2i& imageSize,
                                                    const cv::Point2d& sensorDimensions,
                                                    const int& numberSensorElementsInX,
                                                    double& sx,
                                                    cv::Mat& intrinsic3x3,
                                                    cv::Mat& distortion1x4,
                                                    cv::Mat& rvec1x3,
                                                    cv::Mat& tvec1x3,
                                                    const bool& fullOptimisation = false
                                                   );

/**
* \brief Performs a stereo camera calibration using Tsai 1987 method.
* If ITK is not compiled in, you just have the initial linear bit.
* Will check for planarity and then switch to either the coplanar or the non-coplanar method.
* This method computes Tsai 1987 for left and right camera, and then optimises
* the full intrinsic (2 x 4 DOF), distortion (2 x 1 DOF) and extrinsic (2 x 6 DOF) = 22 DOF.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double TsaiStereoCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                                              const cv::Size2i& imageSize,
                                                              const niftk::PointSet& points2DLeft,
                                                              const cv::Point2d& sensorDimensionsLeft,
                                                              const int& numberSensorElementsInXLeft,
                                                              double& sensorScaleInXLeft,
                                                              cv::Mat& intrinsic3x3Left,
                                                              cv::Mat& distortion1x4Left,
                                                              cv::Mat& rvec1x3Left,
                                                              cv::Mat& tvec1x3Left,
                                                              const niftk::PointSet& points2DRight,
                                                              const cv::Point2d& sensorDimensionsRight,
                                                              const int& numberSensorElementsInXRight,
                                                              double& sensorScaleInXRight,
                                                              cv::Mat& intrinsic3x3Right,
                                                              cv::Mat& distortion1x4Right,
                                                              cv::Mat& rvec1x3Right,
                                                              cv::Mat& tvec1x3Right,
                                                              cv::Mat& leftToRightRotationMatrix3x3,
                                                              cv::Mat& leftToRightTranslationVector3x1,
                                                              const bool& fullOptimisation = false
                                                             );

} // end namespace

#endif
