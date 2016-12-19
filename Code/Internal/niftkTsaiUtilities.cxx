/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTsaiUtilities_p.h"
#include "niftkNiftyCalExceptionMacro.h"

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

  if (distortion.rows != 1 || distortion.cols != 5)
  {
    distortion = cvCreateMat(1, 5, CV_64FC1);
  }
  distortion = cv::Mat::zeros(1, 5, CV_64FC1);

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
std::vector<cv::Point2d> NormalisePoints(const std::vector<cv::Point2d>& points2D,
                                         const double& dxPrime,
                                         const cv::Point2d& imageCentre,
                                         const cv::Point2d& sensorDimensions,
                                         const double& sx
                                        )
{
  unsigned int numberOfPoints = points2D.size();

  std::vector<cv::Point2d> result(numberOfPoints);

  // Step (a)(iv) - compute real image coordinates.
  for (int i = 0; i < numberOfPoints; i++)
  {
    result[i].x =            dxPrime*(points2D[i].x - imageCentre.x)/sx;
    result[i].y = sensorDimensions.y*(points2D[i].y - imageCentre.y);
  }

  return result;
}


//-----------------------------------------------------------------------------
cv::Mat CalculateEquation10(const std::vector<cv::Point3d>& points3D,
                            const std::vector<cv::Point2d>& points2D
                           )
{
  unsigned int numberOfPoints = points2D.size();
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
  return X;
}


//-----------------------------------------------------------------------------
double CalculateTySquaredForCoplanar(const cv::Mat& X)
{
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
  return TySquared;
}


//-----------------------------------------------------------------------------
void CalculateTxAndTy(const std::vector<cv::Point3d>& points3D,
                      const std::vector<cv::Point2d>& points2D,
                      const cv::Mat& X,
                      const double& TySquared,
                      double& Tx,
                      double& Ty
                     )
{

  unsigned int numberOfPoints = points2D.size();

  // Pick the object point furthest from the image centre.
  double bestDistanceSoFar = 0;
  double distance = 0;
  int bestIndexSoFar = -1;
  for (int i = 0; i < numberOfPoints; i++)
  {
    distance = sqrt(  (points2D[i].x * points2D[i].x)
                    + (points2D[i].y * points2D[i].y)
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

  Ty = sqrt(TySquared); // (c)(ii) - let Ty be positive.

  // (c)(iii) - calculate x, y.
  Tx = X.at<double>(2, 0)*Ty;
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
}


//-----------------------------------------------------------------------------
void CalculateApproximateFAndTz(const cv::Mat& R,
                                const std::vector<cv::Point3d>& points3D,
                                const std::vector<cv::Point2d>& points2D,
                                const double& sensorDy,
                                const double& Ty,
                                double& f,
                                double& Tz)
{
  int numberOfPoints = points2D.size();
  cv::Mat A = cvCreateMat ( numberOfPoints, 2, CV_64FC1 );
  cv::Mat B = cvCreateMat ( numberOfPoints, 1, CV_64FC1 );

  for (int i = 0; i < numberOfPoints; i++)
  {
    A.at<double>(i, 0) = R.at<double>(1,0)*points3D[i].x + R.at<double>(1,1)*points3D[i].y + R.at<double>(1,2)*points3D[i].z + Ty;
    A.at<double>(i, 1) = -1.0*sensorDy*points2D[i].y;
    B.at<double>(i, 0) = (R.at<double>(2,0)*points3D[i].x + R.at<double>(2,1)*points3D[i].y + R.at<double>(2,2)*points3D[i].z)*sensorDy*points2D[i].y;
  }

  cv::Mat pseudoInverse = cvCreateMat(2, numberOfPoints, CV_64FC1);
  cv::invert(A, pseudoInverse, CV_SVD);
  cv::Mat X = pseudoInverse * B;

  f = X.at<double>(0, 0);
  Tz = X.at<double>(1, 0);
}


//-----------------------------------------------------------------------------
void CalculateMoreOrthonormalR(cv::Mat& R)
{
  double r1 = R.at<double>(0, 0);
  double r2 = R.at<double>(0, 1);
  double r3 = R.at<double>(0, 2);
  double r4 = R.at<double>(1, 0);
  double r5 = R.at<double>(1, 1);
  double r6 = R.at<double>(1, 2);
  double r7 = R.at<double>(2, 0);

  double gamma = atan2 (r4, r1);
  double sg = sin(gamma);
  double cg = cos(gamma);

  double beta = atan2 (-r7, r1 * cg + r4 * sg);
  double sb = sin(beta);
  double cb = cos(beta);

  double alpha = atan2 (r3 * sg - r6 * cg, r5 * cg - r2 * sg);
  double sa = sin(alpha);
  double ca = cos(alpha);

  R.at<double>(0, 0) = cb * cg;
  R.at<double>(0, 1) = cg * sa * sb - ca * sg;
  R.at<double>(0, 2) = sa * sg + ca * cg * sb;
  R.at<double>(1, 0) = cb * sg;
  R.at<double>(1, 1) = sa * sb * sg + ca * cg;
  R.at<double>(1, 2) = ca * sb * sg - cg * sa;
  R.at<double>(2, 0) = -sb;
  R.at<double>(2, 1) = cb * sa;
  R.at<double>(2, 2) = ca * cb;
}


//-----------------------------------------------------------------------------
void CalculateRForCoplanar(const cv::Mat& X,
                           const double& Ty,
                           cv::Mat& R)
{
  // (3)(ii) - compute equation (14a).
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

  CalculateMoreOrthonormalR(R);
}


//-----------------------------------------------------------------------------
void CalculateRWithFAndTz(const std::vector<cv::Point3d>& points3D,
                          const std::vector<cv::Point2d>& points2D,
                          const cv::Point2d& sensorDimensions,
                          const double& Ty,
                          cv::Mat& R,
                          double& Tz,
                          double& f
                         )
{
  niftk::CalculateApproximateFAndTz(R, points3D, points2D, sensorDimensions.y, Ty, f, Tz);

  // (3)(iii) - Equation (14b)
  if (f < 0)
  {
    R.at<double>(0, 2) = -R.at<double>(0, 2);
    R.at<double>(1, 2) = -R.at<double>(1, 2);
    R.at<double>(2, 0) = -R.at<double>(2, 0);
    R.at<double>(2, 1) = -R.at<double>(2, 1);
    niftk::CalculateApproximateFAndTz(R, points3D, points2D, sensorDimensions.y, Ty, f, Tz);

    if (f < 0)
    {
      niftkNiftyCalThrow() << "Possible handedness problem with data.";
    }
  }
}


//-----------------------------------------------------------------------------
cv::Mat CalculateEquation16(const std::vector<cv::Point3d>& points3D,
                            const std::vector<cv::Point2d>& points2D)
{
  unsigned int numberOfPoints = points2D.size();
  cv::Mat A = cvCreateMat ( numberOfPoints, 7, CV_64FC1 );
  cv::Mat B = cvCreateMat ( numberOfPoints, 1, CV_64FC1 );

  // Section G, NonCoplanar, Stage 1, Part (b) - implement and solve equation (16).
  for (int i = 0; i < numberOfPoints; i++)
  {
    A.at<double>(i, 0) =   points2D[i].y * points3D[i].x;
    A.at<double>(i, 1) =   points2D[i].y * points3D[i].y;
    A.at<double>(i, 2) =   points2D[i].y * points3D[i].z;
    A.at<double>(i, 3) =   points2D[i].y;
    A.at<double>(i, 4) = -(points2D[i].x * points3D[i].x);
    A.at<double>(i, 5) = -(points2D[i].x * points3D[i].y);
    A.at<double>(i, 6) = -(points2D[i].x * points3D[i].z);
    B.at<double>(i, 0) =   points2D[i].x;
  }

  cv::Mat pseudoInverse = cvCreateMat(7, numberOfPoints, CV_64FC1);
  cv::invert(A, pseudoInverse, CV_SVD);
  cv::Mat X = pseudoInverse * B;
  return X;
}


//-----------------------------------------------------------------------------
double CalculateAbsTyForNonCoplanar(const cv::Mat& X)
{
  double ty = 1.0/(sqrt(  (X.at<double>(4, 0) * X.at<double>(4, 0))
                        + (X.at<double>(5, 0) * X.at<double>(5, 0))
                        + (X.at<double>(6, 0) * X.at<double>(6, 0))
                       )
                   );
  return ty;
}


//-----------------------------------------------------------------------------
double CalculateSxForNonConplanar(const cv::Mat& X, const double& absTy)
{
  double sx = absTy * sqrt(  (X.at<double>(0, 0) * X.at<double>(0, 0))
                           + (X.at<double>(1, 0) * X.at<double>(1, 0))
                           + (X.at<double>(2, 0) * X.at<double>(2, 0))
                          );
  return sx;
}


//-----------------------------------------------------------------------------
void CalculateRAndTxForNonCoplanar(const cv::Mat& X,  const double& sx, const double& Ty, cv::Mat& R, double& Tx)
{
  double r1 = X.at<double>(0, 0) * Ty / sx;
  double r2 = X.at<double>(1, 0) * Ty / sx;
  double r3 = X.at<double>(2, 0) * Ty / sx;
  Tx = X.at<double>(3, 0) * Ty;
  double r4 = X.at<double>(4, 0) * Ty;
  double r5 = X.at<double>(5, 0) * Ty;
  double r6 = X.at<double>(6, 0) * Ty;
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

  CalculateMoreOrthonormalR(R);
}

} // end namespace
