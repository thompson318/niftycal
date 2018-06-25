/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearCeresMonoOptimiser_h
#define niftkNonLinearCeresMonoOptimiser_h

#include <niftkNiftyCalTypes.h>
#include <cv.h>
#include <list>

namespace niftk
{

NIFTYCAL_WINEXPORT double CeresMonoCameraCalibration(const std::vector<std::vector<cv::Vec3f> >& modelVectors3D,
                                                     const std::vector<std::vector<cv::Vec2f> >& imageVectors2D,
                                                     cv::Mat& intrinsic,
                                                     cv::Mat& distortion,
                                                     std::vector<cv::Mat>& rvecs,
                                                     std::vector<cv::Mat>& tvecs
                                                    );

}

#endif
