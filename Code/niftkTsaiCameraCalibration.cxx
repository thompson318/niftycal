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
#include <vector>

namespace niftk
{

//-----------------------------------------------------------------------------
int signum(const double& x)
{
  if (x < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}


//-----------------------------------------------------------------------------
void AllocateTsaiMatrices(cv::Mat& intrinsic,
                          cv::Mat& distortion,
                          cv::Mat& rvec,
                          cv::Mat& tvec)
{
  if (intrinsic.rows != 3 || intrinsic.cols != 3)
  {
    intrinsic = cvCreateMat(3, 3, CV_64FC1);
  }
  intrinsic = cv::Mat::zeros(3, 3, CV_64FC1);

  if (distortion.rows != 1 || distortion.cols != 4)
  {
    distortion = cvCreateMat(1, 4, CV_64FC1);
  }
  distortion = cv::Mat::zeros(1, 4, CV_64FC1);

  if (rvec.rows != 1 || rvec.cols != 3)
  {
    rvec = cvCreateMat(1, 3, CV_64FC1);
  }
  rvec = cv::Mat::zeros(1, 3, CV_64FC1);

  if (tvec.rows != 1 || tvec.cols != 3)
  {
    tvec = cvCreateMat(1, 3, CV_64FC1);
  }
  tvec = cv::Mat::zeros(1, 3, CV_64FC1);
}


//-----------------------------------------------------------------------------
void CalculateApproximateFAndTz(const cv::Mat& R,
                                const std::vector<cv::Point3d>& points3D,
                                const std::vector<cv::Point2d>& points2D,
                                const double& Ty,
                                double& f,
                                double& Tz)
{
  int numberOfPoints = points2D.size();
  cv::Mat A = cvCreateMat ( numberOfPoints, 2, CV_64FC1 );
  cv::Mat B = cvCreateMat ( numberOfPoints, 1, CV_64FC1 );
  for (int i = 0; i < numberOfPoints; i++)
  {
    A.at<double>(i, 0) = R.at<double>(1,0)*points3D[i].x + R.at<double>(1,1)*points3D[i].y + Ty;
    A.at<double>(i, 1) = -1.0*points2D[i].y;
    B.at<double>(i, 0) = (R.at<double>(2,0)*points3D[i].x + R.at<double>(1,1)*points3D[i].y)*points2D[i].y;
  }

  cv::Mat pseudoInverse = cvCreateMat(2, numberOfPoints, CV_64FC1);
  cv::invert(A, pseudoInverse, CV_SVD);
  cv::Mat X = pseudoInverse * B;

  f = X.at<double>(0, 0);
  Tz = X.at<double>(1, 0);
}


//-----------------------------------------------------------------------------
double TsaiMonoNonCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                            const niftk::PointSet& imagePoints2D,
                                            const cv::Size2i& imageSize,
                                            const cv::Point2d& sensorDimensions,
                                            const int& numberSensorElementsInX,
                                            cv::Mat& intrinsic,
                                            cv::Mat& distortion,
                                            cv::Mat& rvec,
                                            cv::Mat& tvec
                                           )
{
  double rms = 0;
  AllocateTsaiMatrices(intrinsic, distortion, rvec, tvec);

  return rms;
}


//-----------------------------------------------------------------------------
double TsaiMonoCoplanarCameraCalibration(const niftk::Model3D& model3D,
                                         const niftk::PointSet& imagePoints2D,
                                         const cv::Size2i& imageSize,
                                         const cv::Point2d& sensorDimensions,
                                         const int& numberSensorElementsInX,
                                         const double& sx,
                                         cv::Mat& intrinsic,
                                         cv::Mat& distortion,
                                         cv::Mat& rvec,
                                         cv::Mat& tvec
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

  AllocateTsaiMatrices(intrinsic, distortion, rvec, tvec);

  // Step (a)(ii) - implement equation (6d).
  double dxPrime = sensorDimensions.x * static_cast<double>(numberSensorElementsInX) / static_cast<double>(imageSize.width);

  // Step (a)(iii) - pick centre of image memory.
  cv::Point2d imageCentre((imageSize.width -1 ) / 2.0, (imageSize.height - 1) / 2.0);

  // Step (a)(iv) - compute real image coordinates.
  std::vector<cv::Point3d> points3D;
  std::vector<cv::Point2d> points2D;
  std::vector<niftk::NiftyCalIdType> id;
  niftk::ExtractCommonPoints(model3D, imagePoints2D, points3D, points2D, id);
  unsigned int numberOfPoints = points3D.size();

  if (numberOfPoints < 10)
  {
    niftkNiftyCalThrow() << "Too few points. You should have at least 10."; // haven't checked this yet.
  }

  for (int i = 0; i < numberOfPoints; i++)
  {
    points2D[i].x =            dxPrime*(points2D[i].x - imageCentre.x)/sx;
    points2D[i].y = sensorDimensions.y*(points2D[i].y - imageCentre.y);
  }

  cv::Mat A = cvCreateMat ( numberOfPoints, 5, CV_64FC1 );
  cv::Mat B = cvCreateMat ( numberOfPoints, 1, CV_64FC1 );

  // Step (b). Compute 5 unknowns - implement and solve equation (10).
  for (int i = 0; i < numberOfPoints; i++)
  {
    A.at<double>(i, 0) =   points2D[i].y * points3D[i].x;
    A.at<double>(i, 1) =   points2D[i].y * points3D[i].y;
    A.at<double>(i, 2) =   points2D[i].y;
    A.at<double>(i, 3) = -(points2D[i].x * points3D[i].x);
    A.at<double>(i, 4) = -(points2D[i].x * points3D[i].y);
    B.at<double>(i, 0) =   points2D[i].x;
  }
  cv::Mat pseudoInverse = cvCreateMat(5, numberOfPoints, CV_64FC1);
  cv::invert(A, pseudoInverse, CV_SVD);
  cv::Mat X = pseudoInverse * B;

  // Step (c). Compute (r_1, ..., r_9, T_x, T_y).
  // Step (c1). Compute abs(T_y).

  // Equation (11)
  cv::Mat C = cvCreateMat ( 2, 2, CV_64FC1 );
  C.at<double>(0, 0) = X.at<double>(0, 0);
  C.at<double>(0, 1) = X.at<double>(1, 0);
  C.at<double>(1, 0) = X.at<double>(3, 0);
  C.at<double>(1, 1) = X.at<double>(4, 0);
  double TySquared = 0;

  double tol = 0.00000001;
  if (cv::norm(C.row(0)) < tol)
  {
    // Equation (13), row 1.
    TySquared = 1.0/(C.at<double>(1, 0) * C.at<double>(1, 0) + C.at<double>(1, 1) * C.at<double>(1, 1));
  }
  else if (cv::norm(C.row(1)) < tol)
  {
    // Equation (13), row 0.
    TySquared = 1.0/(C.at<double>(0, 0) * C.at<double>(0, 0) + C.at<double>(0, 1) * C.at<double>(0, 1));
  }
  else if (cv::norm(C.col(0)) < tol)
  {
    // Equation (13), col 1.
    TySquared = 1.0/(C.at<double>(0, 1) * C.at<double>(0, 1) + C.at<double>(1, 1) * C.at<double>(1, 1));
  }
  else if (cv::norm(C.col(1)) < tol)
  {
    // Equation (13), col 0.
    TySquared = 1.0/(C.at<double>(0, 0) * C.at<double>(0, 0) + C.at<double>(1, 0) * C.at<double>(1, 0));
  }
  else
  {
    // Equation (12).
    double Sr = C.at<double>(0, 0)*C.at<double>(0, 0)
              + C.at<double>(0, 1)*C.at<double>(0, 1)
              + C.at<double>(1, 0)*C.at<double>(1, 0)
              + C.at<double>(1, 1)*C.at<double>(1, 1);

    double r1r5mr4r2Squared = (C.at<double>(0, 0) * C.at<double>(1, 1) - C.at<double>(1, 0) * C.at<double>(0, 1))
                             *(C.at<double>(0, 0) * C.at<double>(1, 1) - C.at<double>(1, 0) * C.at<double>(0, 1));

    TySquared = (Sr - sqrt(Sr*Sr - 4.0*r1r5mr4r2Squared))
                / (2.0 * r1r5mr4r2Squared);
  }

  // Pick the object point furthest from the image centre.
  double bestDistanceSoFar = 0;
  double distance = 0;
  int bestIndexSoFar = -1;
  for (int i = 0; i < numberOfPoints; i++)
  {
    distance = sqrt(  (points2D[i].x - imageCentre.x) * (points2D[i].x - imageCentre.x)
                    + (points2D[i].y - imageCentre.y) * (points2D[i].y - imageCentre.y)
                   );
    if (distance > bestDistanceSoFar)
    {
      bestIndexSoFar = i;
      bestDistanceSoFar = distance;
    }
  }
  if (bestIndexSoFar == -1 || bestIndexSoFar == numberOfPoints)
  {
    niftkNiftyCalThrow() << "Failed to find best object point furthest from image centre.";
  }

  double Ty = sqrt(TySquared); // (c)(ii) - let Ty be positive.

  // (c)(iii) - calculate x, y.
  double Tx = X.at<double>(2, 0)*Ty;
  double  x = X.at<double>(0, 0)*Ty*points3D[bestIndexSoFar].x
            + X.at<double>(1, 0)*Ty*points3D[bestIndexSoFar].y
            + Tx;
  double  y = X.at<double>(3, 0)*Ty*points3D[bestIndexSoFar].x
            + X.at<double>(4, 0)*Ty*points3D[bestIndexSoFar].y
            + Ty;

  // (c)(iv) - set the sign of Ty.
  if (   signum(x) != signum(points2D[bestIndexSoFar].x)
      || signum(y) != signum(points2D[bestIndexSoFar].y))
  {
    Ty = -1.0 * Ty;
    Tx = X.at<double>(2, 0)*Ty;
  }

  // (3)(ii) - compute equation (14a).
  cv::Mat R = cvCreateMat ( 3, 3, CV_64FC1 );
  double r1 = X.at<double>(0, 0) * Ty;
  double r2 = X.at<double>(1, 0) * Ty;
  double r3 = sqrt(1.0 - r1*r1 - r2*r2);
  double r4 = X.at<double>(3, 0) * Ty;
  double r5 = X.at<double>(4, 0) * Ty;
  double s = -1.0 * signum(r1*r4 + r2*r5);
  double r6 = s * sqrt(1.0 - r4*r4 - r5*r5);
  double r7 = r2*r6 - r5*r3;
  double r8 = -1.0*(r1*r6 - r4*r3);
  double r9 = r1*r5 - r4*r2;
  R.at<double>(0, 0) = r1;
  R.at<double>(0, 1) = r2;
  R.at<double>(0, 2) = r3;
  R.at<double>(1, 0) = r4;
  R.at<double>(1, 1) = r5;
  R.at<double>(1, 2) = r6;
  R.at<double>(2, 0) = r7;
  R.at<double>(2, 1) = r8;
  R.at<double>(2, 2) = r9;

  // Stage 2 - F, Tz, k.
  double f = 0;
  double Tz = 0;
  double k1 = 0;
  niftk::CalculateApproximateFAndTz(R, points3D, points2D, Tx, f, Tz);

  // (3)(iii) - Equation (14b)
  if (f < 0)
  {
    R.at<double>(0, 2) = -R.at<double>(0, 2);
    R.at<double>(1, 2) = -R.at<double>(1, 2);
    R.at<double>(2, 0) = -R.at<double>(2, 0);
    R.at<double>(2, 1) = -R.at<double>(2, 1);
    niftk::CalculateApproximateFAndTz(R, points3D, points2D, Tx, f, Tz);

    if (f < 0)
    {
      niftkNiftyCalThrow() << "Possible handedness problem with data.";
    }
  }

  // (e): Insert non-linear optimisation of f, Tz and K1 here.

  // Prepare output.
  intrinsic.at<double>(0, 0) = f;
  intrinsic.at<double>(0, 1) = 0;
  intrinsic.at<double>(0, 2) = imageCentre.x;
  intrinsic.at<double>(1, 0) = 0;
  intrinsic.at<double>(1, 1) = f * sx;        // sx not optimised.
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

  double rms = niftk::ComputeRMSProjectionError(model3D, imagePoints2D, intrinsic, distortion, rvec, tvec);
  return rms;
}

} // end namespace
