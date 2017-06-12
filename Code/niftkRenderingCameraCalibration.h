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
#include "niftkIPoint2DDetector.h"
#include <vector>
#include <cv.h>

namespace niftk
{

/**
* \file niftkRenderingCameraCalibration.h
* \brief Calibration routines based on rendering a model.
* \ingroup calibration
*/

/**
* \brief Performs a mono camera calibration using rendering of a model.
* \return rms re-projection error.
*/
NIFTYCAL_WINEXPORT double RenderingMonoCameraCalibration(const std::string& modelFileName,
                                                         const std::string& textureFileName,
                                                         const std::vector<cv::Mat>& images,
                                                         const cv::Size2i& imageSize,
                                                         cv::Mat& intrinsic,
                                                         cv::Mat& distortion,
                                                         std::vector<cv::Mat>& rvecs,
                                                         std::vector<cv::Mat>& tvecs
                                                        );

/**
* \brief Performs a stereo camera calibration using rendering of a model.
* \return rms re-projection error (pix) and re-construction error (mm).
*/
NIFTYCAL_WINEXPORT cv::Matx21d RenderingStereoCameraCalibration(const std::string& modelFileName,
                                                                const std::string& textureFileName,
                                                                const std::vector<PointSet>& leftImages,
                                                                const std::vector<PointSet>& rightImages,
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
