/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingCameraCalibration.h"

namespace niftk
{

//-----------------------------------------------------------------------------
void RenderingMonoCameraCalibration(const std::string& modelFileName,
                                    const std::string& textureFileName,
                                    const std::vector<cv::Mat>& images,
                                    const cv::Size2i& imageSize,
                                    cv::Mat& intrinsic,
                                    cv::Mat& distortion,
                                    std::vector<cv::Mat>& rvecs,
                                    std::vector<cv::Mat>& tvecs
                                   )
{
  // Algorithm
  // While still improving.
  //   1. Optimise intrinsic/distortion, as you render ONCE at each pose,
  //   then repeatedly undistort/unwarp the distorted image to match.
  //   2. Optimise extrinsic, as you unwarp once for each image,
  //   then repeatedly render the model to match the images.
  // end
}


//-----------------------------------------------------------------------------
void RenderingStereoCameraCalibration(const std::string& modelFileName,
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
                                     )
{
  // Algorithm
  // While still improving.
  //   1. Optimise intrinsic/distortion for left camera
  //   2. Optimise intrinsic/distortion for right camera
  //   3. Optimise extrinsic for left and right simultanously.
  // end
}

} // end namespace
