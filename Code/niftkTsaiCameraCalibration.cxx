/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTsaiCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"
#include "Internal/niftkTsaiUtilities_p.h"
#include "Internal/niftkNonLinearTsai3ParamOptimiser.h"
#include "Internal/niftkNonLinearTsai5ParamOptimiser.h"
#include "Internal/niftkNonLinearTsai8ParamOptimiser.h"
#include "Internal/niftkNonLinearTsai10ParamOptimiser.h"
#include "Internal/niftkNonLinearTsai11ParamOptimiser.h"
#include <vector>

namespace niftk
{

//-----------------------------------------------------------------------------
void TsaiNonLinearOptimisation(const niftk::Model3D& model3D,
                               const niftk::PointSet& imagePoints2D,
                               const double& dxPrime,
                               const cv::Point2d& sensorDimensions,
                               double& sx,
                               double& Tz,
                               double& f,
                               double& k1,
                               cv::Point2d& imageCentre,
                               cv::Mat& intrinsic,
                               cv::Mat& distortion,
                               cv::Mat& rvec,
                               cv::Mat& tvec,
                               const bool& fullOptimisation
                              )
{
#ifdef NIFTYCAL_WITH_ITK

  cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

  std::list<niftk::PointSet> listOfPoints;
  listOfPoints.push_back(imagePoints2D);

  // (e): Non-linear optimisation of f, Tz and K1.
  niftk::NonLinearTsai3ParamOptimiser::Pointer tsai3Param = niftk::NonLinearTsai3ParamOptimiser::New();
  tsai3Param->SetModel(&model3D);
  tsai3Param->SetPoints(&listOfPoints);
  tsai3Param->SetIntrinsic(&intrinsic);
  tsai3Param->SetExtrinsic(&extrinsic);
  tsai3Param->Optimise(Tz, f, k1);
  tvec.at<double>(0, 2) = Tz;
  intrinsic.at<double>(0, 0) = f * sx;
  intrinsic.at<double>(1, 1) = f;
  distortion.at<double>(0, 0) = k1;

  if (fullOptimisation)
  {
    // (e): Non-linear optimisation of f, Tz, K1, Cx and Cy.
    niftk::NonLinearTsai5ParamOptimiser::Pointer tsai5Param = niftk::NonLinearTsai5ParamOptimiser::New();
    tsai5Param->SetModel(&model3D);
    tsai5Param->SetPoints(&listOfPoints);
    tsai5Param->SetCameraConstants(dxPrime, sensorDimensions, sx);
    tsai5Param->Optimise(Tz, f, k1, imageCentre.x, imageCentre.y);
    tvec.at<double>(0, 2) = Tz;
    intrinsic.at<double>(0, 0) = f * sx;
    intrinsic.at<double>(1, 1) = f;
    intrinsic.at<double>(0, 2) = imageCentre.x;
    intrinsic.at<double>(1, 2) = imageCentre.y;
    distortion.at<double>(0, 0) = k1;

    // (e): Non-linear optimisation of Rx, Ry, Rz, Tx, Ty, Tz, f and k1.
    niftk::NonLinearTsai8ParamOptimiser::Pointer tsai8Param = niftk::NonLinearTsai8ParamOptimiser::New();
    tsai8Param->SetModel(&model3D);
    tsai8Param->SetPoints(&listOfPoints);
    tsai8Param->SetIntrinsic(&intrinsic);
    tsai8Param->Optimise(rvec.at<double>(0, 0), rvec.at<double>(0, 1), rvec.at<double>(0, 2),
                         tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2),
                         f, k1);
    distortion.at<double>(0, 0) = k1;
    intrinsic.at<double>(0, 0) = f * sx;
    intrinsic.at<double>(1, 1) = f;

    // (e): Non-linear optimisation of Rx, Ry, Rz, Tx, Ty, Tz, f, k1, Cx and Cy.
    niftk::NonLinearTsai10ParamOptimiser::Pointer tsai10Param = niftk::NonLinearTsai10ParamOptimiser::New();
    tsai10Param->SetModel(&model3D);
    tsai10Param->SetPoints(&listOfPoints);
    tsai10Param->Optimise(rvec.at<double>(0, 0), rvec.at<double>(0, 1), rvec.at<double>(0, 2),
                          tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2),
                          f,
                          intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2),
                          distortion.at<double>(0, 0));
    intrinsic.at<double>(0, 0) = f * sx;
    intrinsic.at<double>(1, 1) = f;
  }

#endif
}


//-----------------------------------------------------------------------------
double TsaiMonoNonCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                            const niftk::PointSet& imagePoints2D,
                                            const cv::Size2i& imageSize,
                                            const cv::Point2d& sensorDimensions,
                                            const int& numberSensorElementsInX,
                                            double& sx,
                                            cv::Mat& intrinsic,
                                            cv::Mat& distortion,
                                            cv::Mat& rvec,
                                            cv::Mat& tvec,
                                            const bool& fullOptimisation
                                           )
{
  // Give up if all 3D points have a z component equal to zero.
  bool foundNonZero = false;
  niftk::Model3D::const_iterator iter;
  for (iter = model3D.begin(); iter != model3D.end(); iter++)
  {
    if (iter->second.point.z != 0)
    {
      foundNonZero = true;
    }
  }
  if (!foundNonZero)
  {
    niftkNiftyCalThrow() << "For Tsai's non-coplanar method, z can't all be zero";
  }

  // This is for consistencies sake, as other OpenCV routines self-allocate arrays.
  AllocateTsaiMatrices(intrinsic, distortion, rvec, tvec);

  // Step (a)(ii) - implement equation (6d).
  double dxPrime = sensorDimensions.x * static_cast<double>(numberSensorElementsInX) / static_cast<double>(imageSize.width);

  // Step (a)(iii) - pick centre of image memory to start with.
  cv::Point2d imageCentre((imageSize.width -1 ) / 2.0, (imageSize.height - 1) / 2.0);

  // Extract points into a more usable format.
  std::vector<cv::Point3d> points3D;
  std::vector<cv::Point2d> points2D;
  std::vector<niftk::NiftyCalIdType> id;
  niftk::ExtractCommonPoints(model3D, imagePoints2D, points3D, points2D, id);
  unsigned int numberOfPoints = points3D.size();

  // Check for minimum points.
  if (numberOfPoints < 7)
  {
    niftkNiftyCalThrow() << "Too few points. You should have at least 7."; // haven't checked this yet.
  }

  // Step (a)(iv) - Normalises points with respect to image centre.
  points2D = niftk::NormalisePoints(points2D, dxPrime, imageCentre, sensorDimensions, sx);

  cv::Mat X = niftk::CalculateEquation16(points3D, points2D);

  // Equation (17)
  double AbsTy = niftk::CalculateAbsTyForNonCoplanar(X);

  // Procedure for setting sign of Ty is same as co-planar case.
  double Tx = 0;
  double Ty = 0;
  niftk::CalculateTxAndTy(points3D, points2D, X, AbsTy*AbsTy, Tx, Ty);

  // Equation (18)
  sx = niftk::CalculateSxForNonConplanar(X, AbsTy);

  // Stage 2 - F, Tz, k.
  double k1 = 0;
  double f = 0;
  double Tz = 0;
  cv::Mat R = cvCreateMat ( 3, 3, CV_64FC1 );
  niftk::CalculateRForNonCoplanar(X, sx, Ty, R, Tx);
  niftk::CalculateRWithFAndTz(points3D, points2D, sensorDimensions, Ty, R, Tz, f);

  // Initial guess.
  intrinsic.at<double>(0, 0) = f * sx;
  intrinsic.at<double>(0, 1) = 0;
  intrinsic.at<double>(0, 2) = imageCentre.x;
  intrinsic.at<double>(1, 0) = 0;
  intrinsic.at<double>(1, 1) = f;
  intrinsic.at<double>(1, 2) = imageCentre.y;
  intrinsic.at<double>(2, 0) = 0;
  intrinsic.at<double>(2, 1) = 0;
  intrinsic.at<double>(2, 2) = 1;

  distortion.at<double>(0, 0) = k1;
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  tvec.at<double>(0, 0) = Tx;
  tvec.at<double>(0, 1) = Ty;
  tvec.at<double>(0, 2) = Tz;

  cv::Rodrigues(R, rvec);

  niftk::TsaiNonLinearOptimisation(model3D, imagePoints2D, dxPrime, sensorDimensions, sx, Tz, f, k1, imageCentre, intrinsic, distortion, rvec, tvec, fullOptimisation);

  double rms = niftk::ComputeRMSProjectionError(model3D, imagePoints2D, intrinsic, distortion, rvec, tvec);
  return rms;
}


//-----------------------------------------------------------------------------
double TsaiMonoCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                         const niftk::PointSet& imagePoints2D,
                                         const cv::Size2i& imageSize,
                                         const cv::Point2d& sensorDimensions,
                                         const int& numberSensorElementsInX,
                                         double& sx,
                                         cv::Mat& intrinsic,
                                         cv::Mat& distortion,
                                         cv::Mat& rvec,
                                         cv::Mat& tvec,
                                         const bool& fullOptimisation
                                        )
{
  // Give up if all 3D points do not have a z component equal to zero.
  niftk::Model3D::const_iterator iter;
  for (iter = model3D.begin(); iter != model3D.end(); iter++)
  {
    if (iter->second.point.z != 0)
    {
      niftkNiftyCalThrow() << "For Tsai's coplanar method, z must be zero, and id["
                           << iter->second.id << "]=" << iter->second.point << ", isn't";
    }
  }

  // This is for consistencies sake, as other OpenCV routines self-allocate arrays.
  AllocateTsaiMatrices(intrinsic, distortion, rvec, tvec);

  // Step (a)(ii) - implement equation (6d).
  double dxPrime = sensorDimensions.x * static_cast<double>(numberSensorElementsInX) / static_cast<double>(imageSize.width);

  // Step (a)(iii) - pick centre of image memory to start with.
  cv::Point2d imageCentre((imageSize.width -1 ) / 2.0, (imageSize.height - 1) / 2.0);

  // Extract points into a more usable format.
  std::vector<cv::Point3d> points3D;
  std::vector<cv::Point2d> points2D;
  std::vector<niftk::NiftyCalIdType> id;
  niftk::ExtractCommonPoints(model3D, imagePoints2D, points3D, points2D, id);
  unsigned int numberOfPoints = points3D.size();

  // Check for minimum points.
  if (numberOfPoints < 7)
  {
    niftkNiftyCalThrow() << "Too few points. You should have at least 7."; // haven't checked this yet.
  }

  // Step (a)(iv) - Normalises points with respect to image centre.
  points2D = niftk::NormalisePoints(points2D, dxPrime, imageCentre, sensorDimensions, sx);

  cv::Mat X = niftk::CalculateEquation10(points3D, points2D);
  double TySquared = niftk::CalculateTySquaredForCoplanar(X);

  double Tx = 0;
  double Ty = 0;
  niftk::CalculateTxAndTy(points3D, points2D, X, TySquared, Tx, Ty);

  // Stage 2 - F, Tz, k.
  double k1 = 0;
  double f = 0;
  double Tz = 0;
  cv::Mat R = cvCreateMat ( 3, 3, CV_64FC1 );

  niftk::CalculateRForCoplanar(X, Ty, R);
  niftk::CalculateRWithFAndTz(points3D, points2D, sensorDimensions, Ty, R, Tz, f);

  // Initial guess.
  intrinsic.at<double>(0, 0) = f * sx;        // sx not optimised.
  intrinsic.at<double>(0, 1) = 0;
  intrinsic.at<double>(0, 2) = imageCentre.x;
  intrinsic.at<double>(1, 0) = 0;
  intrinsic.at<double>(1, 1) = f;
  intrinsic.at<double>(1, 2) = imageCentre.y;
  intrinsic.at<double>(2, 0) = 0;
  intrinsic.at<double>(2, 1) = 0;
  intrinsic.at<double>(2, 2) = 1;

  distortion.at<double>(0, 0) = k1;
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  tvec.at<double>(0, 0) = Tx;
  tvec.at<double>(0, 1) = Ty;
  tvec.at<double>(0, 2) = Tz;

  cv::Rodrigues(R, rvec);

  niftk::TsaiNonLinearOptimisation(model3D, imagePoints2D, dxPrime, sensorDimensions, sx, Tz, f, k1, imageCentre, intrinsic, distortion, rvec, tvec, fullOptimisation);

  double rms = niftk::ComputeRMSProjectionError(model3D, imagePoints2D, intrinsic, distortion, rvec, tvec);
  return rms;
}

} // end namespace
