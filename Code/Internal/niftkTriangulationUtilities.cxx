/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTriangulationUtilities_p.h"
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>

namespace niftk {

//-----------------------------------------------------------------------------
/**
* \brief Triangulates a 3D point using SVD.
*
* Credit to <a href="http://www.morethantechnical.com/2012/01/04/
* simple-triangulation-with-opencv-from-harley-zisserman-w-code/">these authors</a>.
*
*
* \param P1 left camera matrix, meaning a full perspective projection, including extrinsic and intrinsic.
* \param P2 right camera matrix, meaning a full perspective projection, including extrinsic and intrinsic.
* \param u1 normalised left camera image coordinate in pixels.
* \param u2 normalised right camera image coordinate in pixels.
*/
cv::Mat_<double> InternalTriangulatePointUsingSVD(
    const cv::Matx34d& P1,
    const cv::Matx34d& P2,
    const cv::Point3d& u1,
    const cv::Point3d& u2,
    const double& w1,
    const double& w2
    )
{
  // Build matrix A for homogenous equation system Ax = 0
  // Assume X = (x,y,z,1), for Linear-LS method
  // Which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
  cv::Matx43d A((u1.x*P1(2,0)-P1(0,0))/w1, (u1.x*P1(2,1)-P1(0,1))/w1, (u1.x*P1(2,2)-P1(0,2))/w1,
                (u1.y*P1(2,0)-P1(1,0))/w1, (u1.y*P1(2,1)-P1(1,1))/w1, (u1.y*P1(2,2)-P1(1,2))/w1,
                (u2.x*P2(2,0)-P2(0,0))/w2, (u2.x*P2(2,1)-P2(0,1))/w2, (u2.x*P2(2,2)-P2(0,2))/w2,
                (u2.y*P2(2,0)-P2(1,0))/w2, (u2.y*P2(2,1)-P2(1,1))/w2, (u2.y*P2(2,2)-P2(1,2))/w2
               );


  cv::Matx41d B(-(u1.x*P1(2,3) -P1(0,3))/w1,
                -(u1.y*P1(2,3) -P1(1,3))/w1,
                -(u2.x*P2(2,3) -P2(0,3))/w2,
                -(u2.y*P2(2,3) -P2(1,3))/w2
               );

  cv::Mat_<double> X;
  cv::solve(A,B,X,cv::DECOMP_SVD);

  return X;
}


//-----------------------------------------------------------------------------
/**
* \brief Triangulates a 3D point using SVD by calling
* InternalTriangulatePointUsingSVD (above) with different weighting factors.
*
* Credit to <a href="http://www.morethantechnical.com/2012/01/04/
* simple-triangulation-with-opencv-from-harley-zisserman-w-code/">these authors</a>.
*
* \param P1 left camera matrix, meaning a full perspective projection, including extrinsic and intrinsic.
* \param P2 right camera matrix, meaning a full perspective projection, including extrinsic and intrinsic.
* \param u1 normalised left camera image coordinate in pixels.
* \param u2 normalised right camera image coordinate in pixels.
 */
cv::Point3d InternalIterativeTriangulatePointUsingSVD(
    const cv::Matx34d& P1,
    const cv::Matx34d& P2,
    const cv::Point3d& u1,
    const cv::Point3d& u2
    )
{
  double epsilon = 0.00000000001;
  double w1 = 1;
  double w2 = 1;
  cv::Mat_<double> X(4,1);

  for (int i = 0; i < 10; i++) // Hartley suggests 10 iterations at most
  {
    cv::Mat_<double> X_ = InternalTriangulatePointUsingSVD(P1,P2,u1,u2,w1,w2);
    X(0) = X_(0);
    X(1) = X_(1);
    X(2) = X_(2);
    X(3) = 1.0;

    double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
    double p2x2 = cv::Mat_<double>(cv::Mat_<double>(P2).row(2)*X)(0);

    if(fabs(w1 - p2x1) <= epsilon && fabs(w2 - p2x2) <= epsilon)
      break;

    w1 = p2x1;
    w2 = p2x2;
  }

  cv::Point3d result;
  result.x = X(0);
  result.y = X(1);
  result.z = X(2);

  return result;
}


//-----------------------------------------------------------------------------
void TriangulatePointPairs(
  const std::vector<cv::Point2f>& leftCameraUndistortedPoints,
  const std::vector<cv::Point2f>& rightCameraUndistortedPoints,
  const cv::Mat& leftCameraIntrinsicParams,
  const cv::Mat& leftCameraRotationVector,
  const cv::Mat& leftCameraTranslationVector,
  const cv::Mat& rightCameraIntrinsicParams,
  const cv::Mat& rightCameraRotationVector,
  const cv::Mat& rightCameraTranslationVector,
  std::vector<cv::Point3f>& outputTriangulatedPoints
  )
{

  if (leftCameraUndistortedPoints.size() != rightCameraUndistortedPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left and right points.";
  }

  int numberOfPoints = leftCameraUndistortedPoints.size();
  outputTriangulatedPoints.clear();
  outputTriangulatedPoints.resize(numberOfPoints);

  cv::Mat K1    = cv::Mat(3, 3, CV_64FC1);
  cv::Mat K2    = cv::Mat(3, 3, CV_64FC1);
  cv::Mat K1Inv = cv::Mat(3, 3, CV_64FC1);
  cv::Mat K2Inv = cv::Mat(3, 3, CV_64FC1);
  cv::Mat R1    = cv::Mat(3, 3, CV_64FC1);
  cv::Mat R2    = cv::Mat(3, 3, CV_64FC1);
  cv::Mat E1    = cv::Mat(4, 4, CV_64FC1);
  cv::Mat E1Inv = cv::Mat(4, 4, CV_64FC1);
  cv::Mat E2    = cv::Mat(4, 4, CV_64FC1);
  cv::Mat L2R   = cv::Mat(4, 4, CV_64FC1);

  // Convert OpenCV stylee rotation vector to a rotation matrix.
  cv::Rodrigues(leftCameraRotationVector, R1);
  cv::Rodrigues(rightCameraRotationVector, R2);

  // Construct:
  // E1 = Object to Left Camera = Left Camera Extrinsics.
  // E2 = Object to Right Camera = Right Camera Extrinsics.
  // K1 = Copy of Left Camera intrinsics.
  // K2 = Copy of Right Camera intrinsics.
  // Copy data into cv::Mat data types.
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      K1.at<double>(i,j) = leftCameraIntrinsicParams.at<double>(i,j);
      K2.at<double>(i,j) = rightCameraIntrinsicParams.at<double>(i,j);
      E1.at<double>(i,j) = R1.at<double>(i,j);
      E2.at<double>(i,j) = R2.at<double>(i,j);
    }
    E1.at<double>(i,3) = leftCameraTranslationVector.at<double>(0,i);
    E2.at<double>(i,3) = rightCameraTranslationVector.at<double>(0,i);
  }
  E1.at<double>(3,0) = 0;
  E1.at<double>(3,1) = 0;
  E1.at<double>(3,2) = 0;
  E1.at<double>(3,3) = 1;
  E2.at<double>(3,0) = 0;
  E2.at<double>(3,1) = 0;
  E2.at<double>(3,2) = 0;
  E2.at<double>(3,3) = 1;

  // We invert the intrinsic params, so we can convert from pixels to normalised image coordinates.
  K1Inv = K1.inv();
  K2Inv = K2.inv();

  // We want output coordinates relative to left camera.
  E1Inv = E1.inv();
  L2R = E2 * E1Inv;

  // Reading Prince 2012 Computer Vision, the projection matrix, is just the extrinsic parameters,
  // as our coordinates will be in a normalised camera space. P1 should be identity, so that
  // reconstructed coordinates are in Left Camera Space, to P2 should reflect a left to right transform.
  cv::Matx34d P1d, P2d;
  P1d(0,0) = 1;
  P1d(0,1) = 0;
  P1d(0,2) = 0;
  P1d(0,3) = 0;
  P1d(1,0) = 0;
  P1d(1,1) = 1;
  P1d(1,2) = 0;
  P1d(1,3) = 0;
  P1d(2,0) = 0;
  P1d(2,1) = 0;
  P1d(2,2) = 1;
  P1d(2,3) = 0;

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      P2d(i,j) = L2R.at<double>(i,j);
    }
  }

  cv::Mat u1    = cv::Mat(3, 1, CV_64FC1);
  cv::Mat u2    = cv::Mat(3, 1, CV_64FC1);
  cv::Mat u1t   = cv::Mat(3, 1, CV_64FC1);
  cv::Mat u2t   = cv::Mat(3, 1, CV_64FC1);

  cv::Point3d u1p, u2p;  // Normalised image coordinates. (i.e. relative to a principal
                         // point of zero, and in millimetres not pixels).
  cv::Point3d r;         // the output 3D point, in reference frame of left camera.

  #pragma omp parallel private(u1), private(u2), private(u1t), private(u2t), private(u1p), private(u2p), private(r), shared(leftCameraUndistortedPoints), shared(rightCameraUndistortedPoints), shared(P1d), shared(P2d)
  {
    #pragma omp for
    for (int i = 0; i < numberOfPoints; i++)
    {
      u1.at<double>(0,0) = leftCameraUndistortedPoints[i].x;
      u1.at<double>(1,0) = leftCameraUndistortedPoints[i].y;
      u1.at<double>(2,0) = 1;

      u2.at<double>(0,0) = rightCameraUndistortedPoints[i].x;
      u2.at<double>(1,0) = rightCameraUndistortedPoints[i].y;
      u2.at<double>(2,0) = 1;

      // Converting to normalised image points
      u1t = K1Inv * u1;
      u2t = K2Inv * u2;

      u1p.x = u1t.at<double>(0,0);
      u1p.y = u1t.at<double>(1,0);
      u1p.z = u1t.at<double>(2,0);

      u2p.x = u2t.at<double>(0,0);
      u2p.y = u2t.at<double>(1,0);
      u2p.z = u2t.at<double>(2,0);

      r = InternalIterativeTriangulatePointUsingSVD(P1d, P2d, u1p, u2p);
      outputTriangulatedPoints[i].x = static_cast<float>(r.x);
      outputTriangulatedPoints[i].y = static_cast<float>(r.y);
      outputTriangulatedPoints[i].z = static_cast<float>(r.z);
    }
  }
}

} // end namespace
