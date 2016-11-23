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
* \brief Performs a mono camera calibration using Tsai 1987 coplanar method.
* If ITK is not compiled in, you just have the initial linear bit.
* \throw Throws niftk::NiftyCalException if z coordinate of any model point is not zero.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double TsaiMonoCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                                            const niftk::PointSet& imagePoints2D,
                                                            const cv::Size2i& imageSize,
                                                            const cv::Point2d& sensorDimensions,
                                                            const int& numberSensorElementsInX,
                                                            double& sx,
                                                            cv::Mat& intrinsic3x3,
                                                            cv::Mat& distortion1x4,
                                                            cv::Mat& rvec1x3,
                                                            cv::Mat& tvec1x3
                                                           );

/**
* \brief Performs a mono camera calibration using Tsai 1987 non co-planar method.
* If ITK is not compiled in, you just have the initial linear bit.
* \throw Throws niftk::NiftyCalException if z coordinate of all model points is zero.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double TsaiMonoNonCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                                               const niftk::PointSet& imagePoints2D,
                                                               const cv::Size2i& imageSize,
                                                               const cv::Point2d& sensorDimensions,
                                                               const int& numberSensorElementsInX,
                                                               double& sx,
                                                               cv::Mat& intrinsic3x3,
                                                               cv::Mat& distortion1x4,
                                                               cv::Mat& rvec1x3,
                                                               cv::Mat& tvec1x3
                                                              );


} // end namespace

#endif
