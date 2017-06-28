/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingCameraCalibration_h
#define niftkRenderingCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include <Internal/niftkIntensityBasedCostFunction.h>
#include <vector>
#include <cv.h>

namespace niftk
{

/**
* \file niftkRenderingCameraCalibration.h
* \brief Intensity based mono/stereo calibration routines.
* \ingroup calibration
*/

/**
* \brief Calibrates the blur found in a video image of a calibration object, using intensity based methods.
*/
NIFTYCAL_WINEXPORT double IntensityBasedBlurringCalibration(niftk::IntensityBasedCostFunction::Pointer blurringCostFunction,
                                                            double& sigma
                                                           );
/**
* \brief Performs a mono camera calibration using intensity based methods.
*/
NIFTYCAL_WINEXPORT double IntensityBasedMonoCameraCalibration(niftk::IntensityBasedCostFunction::Pointer monoIntrinsicCostFunction,
                                                              niftk::IntensityBasedCostFunction::Pointer monoExtrinsicCostFunction,
                                                              cv::Mat& intrinsic,
                                                              cv::Mat& distortion,
                                                              std::vector<cv::Mat>& rvecs,
                                                              std::vector<cv::Mat>& tvecs
                                                             );

/**
* \brief Performs a stereo camera calibration using intensity based methods.
*/
NIFTYCAL_WINEXPORT double IntensityBasedStereoCameraCalibration(niftk::IntensityBasedCostFunction::Pointer monoIntrinsicLeftCostFunction,
                                                                niftk::IntensityBasedCostFunction::Pointer monoIntrinsicRightCostFunction,
                                                                niftk::IntensityBasedCostFunction::Pointer stereoExtrinsicCostFunction,
                                                                cv::Mat& intrinsicLeft,
                                                                cv::Mat& distortionLeft,
                                                                std::vector<cv::Mat>& rvecsLeft,
                                                                std::vector<cv::Mat>& tvecsLeft,
                                                                cv::Mat& intrinsicRight,
                                                                cv::Mat& distortionRight,
                                                                std::vector<cv::Mat>& rvecsRight,
                                                                std::vector<cv::Mat>& tvecsRight,
                                                                cv::Mat& leftToRightRotationMatrix,
                                                                cv::Mat& leftToRightTranslationVector
                                                               );
} // end namespace

#endif
