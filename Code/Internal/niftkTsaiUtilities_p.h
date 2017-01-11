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
* \brief Private (as in 'deliberately not exported') utility functions
* to implement Tsai's 1987 paper on Camera Calibration.
*/

/**
* \brief Ensures matrices are the right size.
* \param intrinsic 3x3 matrix
* \param distortion 1x4 matrix
* \param rvec 1x3 matrix
* \param tvec 1x3 matrix
*/
void AllocateTsaiMatrices(cv::Mat& intrinsic,
                          cv::Mat& distortion,
                          cv::Mat& rvec,
                          cv::Mat& tvec);


/**
* \brief Implements Stage 1 of coplanar case, section (a)(iv)
* to normalise image coordinates w.r.t. centre.
*/
std::vector<cv::Point2d> NormalisePoints(const std::vector<cv::Point2d>& points2D,
                                         const double& dxPrime,
                                         const cv::Point2d& imageCentre,
                                         const cv::Point2d& sensorDimensions,
                                         const double& sx
                                        );


cv::Mat CalculateEquation10(const std::vector<cv::Point3d>& points3D,
                            const std::vector<cv::Point2d>& points2D);


/**
* \brief Implements equation (11), (12) and (13) to get Ty squared.
*/
double CalculateTySquaredForCoplanar(const cv::Mat& X);


/**
* \brief Implements Stage 2 of coplanar case, section (i) to (iv).
*
* Also re-used in non-coplanar case.
*/
void CalculateTxAndTy(const std::vector<cv::Point3d>& points3D,
                      const std::vector<cv::Point2d>& points2D,
                      const cv::Mat& X,
                      const double& TySquared,
                      double& Tx,
                      double& Ty
                     );


/**
* \brief Implements equation (15) to compute approx F and Tz, ignoring lens distortion.
*/
void CalculateApproximateFAndTz(const cv::Mat& R,
                                const std::vector<cv::Point3d>& points3D,
                                const std::vector<cv::Point2d>& points2D,
                                const double& sensorDy,
                                const double& Ty,
                                double& f,
                                double& Tz);


/**
* \brief Implements Stage 3 of coplanar case, section (i) to (iii), equation 14a and 14b.
*/
void CalculateRForCoplanar(const cv::Mat& X,
                           const double& Ty,
                           cv::Mat& R
                          );


/**
* \brief Calls CalculateApproximateFAndTz, then flips R if f < 0
*/
void CalculateRWithFAndTz(const std::vector<cv::Point3d>& points3D,
                          const std::vector<cv::Point2d>& points2D,
                          const cv::Point2d& sensorDimensions,
                          const double& Ty,
                          cv::Mat& R,
                          double& Tz,
                          double& f
                         );

cv::Mat CalculateEquation16(const std::vector<cv::Point3d>& points3D,
                            const std::vector<cv::Point2d>& points2D);


/**
* \brief Implements equation (17), returns positive root of TySquared.
*/
double CalculateAbsTyForNonCoplanar(const cv::Mat& X);


/**
* \brief Implements equation (18).
*/
double CalculateSxForNonConplanar(const cv::Mat& X, const double& absTy);


/**
* \brief Implements Stage 1, part (c), section (4) of non-coplanar case.
*/
void CalculateRAndTxForNonCoplanar(const cv::Mat& X, const double& sx, const double& Ty, cv::Mat& R, double& Tx);

/**
* \brief Calculates a more orthonormal R from Euler angles.
*/
void CalculateMoreOrthonormalR(cv::Mat& R);

} // end namespace

#endif
