/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIterativeCalibrationUtilities_p_h
#define niftkIterativeCalibrationUtilities_p_h

#include <cv.h>

namespace niftk
{

/**
* \file niftkTsaiUtilities_p.h
* \brief Private (as in 'deliberately not exported') utility functions.
*/

int signum(const double& x);


void AllocateTsaiMatrices(cv::Mat& intrinsic,
                          cv::Mat& distortion,
                          cv::Mat& rvec,
                          cv::Mat& tvec);


void CalculateApproximateFAndTz(const cv::Mat& R,
                                const std::vector<cv::Point3d>& points3D,
                                const std::vector<cv::Point2d>& points2D,
                                const double& sensorDy,
                                const double& Ty,
                                double& f,
                                double& Tz);

cv::Mat CalculateEquation10(const std::vector<cv::Point3d>& points3D,
                            const std::vector<cv::Point2d>& points2D
                           );

void CalculateTxAndTy(const std::vector<cv::Point3d>& points3D,
                      const std::vector<cv::Point2d>& points2D,
                      const cv::Mat& X,
                      double& Tx,
                      double& Ty
                     );

void CalculateRWithApproxTzAndF(const std::vector<cv::Point3d>& points3D,
                                const std::vector<cv::Point2d>& points2D,
                                const cv::Point2d& sensorDimensions,
                                const double& Ty,
                                const cv::Mat& X,
                                cv::Mat& R,
                                double& Tz,
                                double& f
                               );

} // end namespace

#endif
