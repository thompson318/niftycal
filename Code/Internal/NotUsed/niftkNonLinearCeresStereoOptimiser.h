/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearCeresStereoOptimiser_h
#define niftkNonLinearCeresStereoOptimiser_h

#include <cv.h>
#include <niftkNiftyCalTypes.h>

namespace niftk
{

NIFTYCAL_WINEXPORT double CeresStereoCameraCalibration(const std::vector<std::vector<cv::Vec3f> >& objectVectors3D,
                                                       const std::vector<std::vector<cv::Vec2f> >& leftVectors2D,
                                                       const std::vector<std::vector<cv::Vec2f> >& rightVectors2D,
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

}

#endif
