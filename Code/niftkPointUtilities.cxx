/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkPointUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <queue>
#include <vector>
#include <functional>
#include <random>

namespace niftk {

//-----------------------------------------------------------------------------
PointSet CopyPoints(const PointSet& p)
{
  PointSet result(p);
  return result;
}


//-----------------------------------------------------------------------------
void CopyPointsInto(const PointSet& a, PointSet& b)
{
  b.clear();

  PointSet::const_iterator iter;
  for (iter = a.begin(); iter != a.end(); ++iter)
  {
    b.insert(IdPoint2D((*iter).first, (*iter).second));
  }
}


//-----------------------------------------------------------------------------
PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor)
{
  PointSet result;

  PointSet::const_iterator iter;
  for (iter = p.begin(); iter != p.end(); ++iter)
  {
    Point2D tmp;
    tmp.point = (*iter).second.point;
    tmp.id = (*iter).second.id;
    tmp.point.x *= scaleFactor.x;
    tmp.point.y *= scaleFactor.y;
    result.insert(IdPoint2D(tmp.id, tmp));
  }

  return result;
}


//-----------------------------------------------------------------------------
void ExtractCommonPoints(const PointSet& inputA,
                         const PointSet& inputB,
                         std::vector<cv::Point2f>& outputA,
                         std::vector<cv::Point2f>& outputB
                         )
{
  outputA.clear();
  outputB.clear();

  niftk::PointSet::const_iterator iterA;
  niftk::PointSet::const_iterator iterB;

  for(iterA = inputA.begin(); iterA != inputA.end(); ++iterA)
  {
    iterB = inputB.find((*iterA).first);
    if (iterB != inputB.end())
    {
      cv::Point2f a;
      a.x = (*iterA).second.point.x;
      a.y = (*iterA).second.point.y;
      outputA.push_back(a);

      cv::Point2f b;
      b.x = (*iterB).second.point.x;
      b.y = (*iterB).second.point.y;
      outputB.push_back(b);
    }
  }
}


//-----------------------------------------------------------------------------
void ExtractCommonPoints(const Model3D& inputA,
                         const Model3D& inputB,
                         std::vector<cv::Point3d>& outputA,
                         std::vector<cv::Point3d>& outputB
                        )
{
  outputA.clear();
  outputB.clear();

  niftk::Model3D::const_iterator iterA;
  niftk::Model3D::const_iterator iterB;

  for(iterA = inputA.begin(); iterA != inputA.end(); ++iterA)
  {
    iterB = inputB.find((*iterA).first);
    if (iterB != inputB.end())
    {
      cv::Point3d a;
      a.x = (*iterA).second.point.x;
      a.y = (*iterA).second.point.y;
      a.z = (*iterA).second.point.z;
      outputA.push_back(a);

      cv::Point3d b;
      b.x = (*iterB).second.point.x;
      b.y = (*iterB).second.point.y;
      b.z = (*iterB).second.point.z;
      outputB.push_back(b);
    }
  }
}


//-----------------------------------------------------------------------------
void ConvertPoints(const PointSet& input,
                   std::vector<cv::Point2f>& outputPoint,
                   std::vector<niftk::NiftyCalIdType>& outputId
                  )
{
  outputPoint.clear();
  outputId.clear();

  niftk::PointSet::const_iterator iter;
  for(iter = input.begin(); iter != input.end(); ++iter)
  {
    cv::Point2f p;
    p.x = (*iter).second.point.x;
    p.y = (*iter).second.point.y;
    outputPoint.push_back(p);
    outputId.push_back((*iter).first);
  }
}


//-----------------------------------------------------------------------------
void ConvertPoints(const std::vector<cv::Point2f>& inputPoint,
                   const std::vector<niftk::NiftyCalIdType>& inputId,
                   PointSet& output
                   )
{
  output.clear();

  if (inputPoint.size() != inputId.size())
  {
    niftkNiftyCalThrow() << "Different number of points and ids.";
  }
  for (size_t i = 0; i < inputPoint.size(); i++)
  {
    niftk::Point2D p2d;
    cv::Point2d p;
    p.x = inputPoint[i].x;
    p.y = inputPoint[i].y;
    p2d.point = p;
    p2d.id = inputId[i];
    output.insert(niftk::IdPoint2D(p2d.id, p2d));
  }
}


//-----------------------------------------------------------------------------
double ComputeRMSDifferenceBetweenMatchingPoints(const PointSet& inputA,
                                                 const PointSet& inputB,
                                                 cv::Point2d& rmsForEachAxis
                                                 )
{
  std::vector<cv::Point2f> a;
  std::vector<cv::Point2f> b;

  niftk::ExtractCommonPoints(inputA, inputB, a, b);
  if (a.size() == 0 || b.size() == 0)
  {
    niftkNiftyCalThrow() << "No common points.";
  }
  if (a.size() != b.size())
  {
    niftkNiftyCalThrow() << "Programming errors, invalid extraction of common points.";
  }

  double rms = 0;
  rmsForEachAxis.x = 0;
  rmsForEachAxis.y = 0;
  cv::Point2d diff;

  for (size_t i = 0; i < a.size(); i++)
  {
    diff.x = a[i].x - b[i].x;
    diff.y = a[i].y - b[i].y;

    rms += ( (diff.x * diff.x)
           + (diff.y * diff.y)
           );
    rmsForEachAxis.x += (diff.x * diff.x);
    rmsForEachAxis.y += (diff.y * diff.y);
  }

  rms /= static_cast<double>(a.size());
  rms = sqrt(rms);

  rmsForEachAxis.x /= static_cast<double>(a.size());
  rmsForEachAxis.y /= static_cast<double>(a.size());

  rmsForEachAxis.x = sqrt(rmsForEachAxis.x);
  rmsForEachAxis.y = sqrt(rmsForEachAxis.y);

  return rms;
}


//-----------------------------------------------------------------------------
double ComputeRMSDifferenceBetweenMatchingPoints(const Model3D& inputA,
                                                 const Model3D& inputB,
                                                 cv::Point3d& rmsForEachAxis
                                                )
{
  std::vector<cv::Point3d> a;
  std::vector<cv::Point3d> b;

  niftk::ExtractCommonPoints(inputA, inputB, a, b);
  if (a.size() == 0 || b.size() == 0)
  {
    niftkNiftyCalThrow() << "No common points.";
  }
  if (a.size() != b.size())
  {
    niftkNiftyCalThrow() << "Programming errors, invalid extraction of common points.";
  }

  double rms = 0;
  rmsForEachAxis.x = 0;
  rmsForEachAxis.y = 0;
  rmsForEachAxis.z = 0;
  cv::Point3d diff;

  for (size_t i = 0; i < a.size(); i++)
  {
    diff.x = (a[i].x - b[i].x);
    diff.y = (a[i].y - b[i].y);
    diff.z = (a[i].z - b[i].z);

    rms += ( (diff.x * diff.x)
           + (diff.y * diff.y)
           + (diff.z * diff.z)
           );

    rmsForEachAxis.x += diff.x;
    rmsForEachAxis.y += diff.y;
    rmsForEachAxis.z += diff.z;

  }
  rms /= static_cast<double>(a.size());
  rms = sqrt(rms);

  rmsForEachAxis.x /= static_cast<double>(a.size());
  rmsForEachAxis.y /= static_cast<double>(a.size());
  rmsForEachAxis.z /= static_cast<double>(a.size());

  rmsForEachAxis.x = sqrt(rmsForEachAxis.x);
  rmsForEachAxis.y = sqrt(rmsForEachAxis.y);
  rmsForEachAxis.z = sqrt(rmsForEachAxis.z);

  return rms;
}


//-----------------------------------------------------------------------------
void UndistortPoints(const PointSet& distortedPoints,
                     const cv::Mat& cameraIntrinsics,
                     const cv::Mat& distortionCoefficients,
                     PointSet& undistortedPoints
                    )
{

  undistortedPoints.clear();

  std::vector<cv::Point2f> distorted;
  std::vector<niftk::NiftyCalIdType> ids;
  niftk::ConvertPoints(distortedPoints, distorted, ids);

  std::vector<cv::Point2f> undistorted(distorted.size());
  cv::undistortPoints(distorted, undistorted, cameraIntrinsics, distortionCoefficients, cv::Mat(), cameraIntrinsics);

  niftk::ConvertPoints(undistorted, ids, undistortedPoints);
}


//-----------------------------------------------------------------------------
void UndistortPoints(const std::vector<PointSet>& distortedPoints,
                     const cv::Mat& cameraIntrinsics,
                     const cv::Mat& distortionCoefficients,
                     std::vector<PointSet>& undistortedPoints
                     )

{
  undistortedPoints.clear();

  for (size_t i = 0; i < distortedPoints.size(); i++)
  {
    PointSet uP;
    UndistortPoints(distortedPoints[i], cameraIntrinsics, distortionCoefficients, uP);
    undistortedPoints.push_back(uP);
  }
}


//-----------------------------------------------------------------------------
void DistortPoints(const PointSet& undistortedPoints,
                   const cv::Mat& cameraIntrinsics,
                   const cv::Mat& distortionCoefficients,
                   PointSet& distortedPoints
                  )
{
  std::vector<cv::Point2f> undistorted;
  std::vector<cv::Point2f> distorted;
  std::vector<niftk::NiftyCalIdType> ids;

  niftk::ConvertPoints(undistortedPoints, undistorted, ids);

  // see: http://stackoverflow.com/questions/21615298/opencv-distort-back
  for (size_t i = 0; i < undistorted.size(); i++)
  {
    cv::Point2f relative;
    relative.x = (undistorted[i].x - cameraIntrinsics.at<double>(0,2))/cameraIntrinsics.at<double>(0,0);
    relative.y = (undistorted[i].y - cameraIntrinsics.at<double>(1,2))/cameraIntrinsics.at<double>(1,1);

    double r2 = relative.x*relative.x + relative.y*relative.y;
    double radial = (1 + distortionCoefficients.at<double>(0,0) * r2
                     + distortionCoefficients.at<double>(0,1) * r2 * r2 );

    cv::Point2d dist;
    dist.x = relative.x * radial;
    dist.y = relative.y * radial;

    dist.x = dist.x + (2 * distortionCoefficients.at<double>(0,2) * relative.x * relative.y
                       + distortionCoefficients.at<double>(0,3) * (r2 + 2 * relative.x * relative.x));
    dist.y = dist.y + (distortionCoefficients.at<double>(0,2) * (r2 + 2 * relative.y * relative.y)
                       + 2 * distortionCoefficients.at<double>(0,3) * relative.x * relative.y);

    dist.x = dist.x * cameraIntrinsics.at<double>(0,0) + cameraIntrinsics.at<double>(0,2);
    dist.y = dist.y * cameraIntrinsics.at<double>(1,1) + cameraIntrinsics.at<double>(1,2);

    distorted.push_back(dist);
  }
  niftk::ConvertPoints(distorted, ids, distortedPoints);
}


//-----------------------------------------------------------------------------
void DistortPoints(const std::vector<PointSet>& undistortedPoints,
                   const cv::Mat& cameraIntrinsics,
                   const cv::Mat& distortionCoefficients,
                   std::vector<PointSet>& distortedPoints
                   )
{
  distortedPoints.clear();

  for (size_t i = 0; i < undistortedPoints.size(); i++)
  {
    PointSet dp;
    DistortPoints(undistortedPoints[i], cameraIntrinsics, distortionCoefficients, dp);
    distortedPoints.push_back(dp);
  }
}


//-----------------------------------------------------------------------------
PointSet TrimPoints(const PointSet& input,
                    const PointSet& reference,
                    const float& percentage
                   )
{
  PointSet result;

  typedef std::pair<double, niftk::NiftyCalIdType> P;
  std::priority_queue< P, std::vector<P>, std::greater<P> > queue;

  PointSet::const_iterator iter;
  for (iter = input.begin(); iter != input.end(); ++iter)
  {
    PointSet::const_iterator refIter = reference.find((*iter).first);
    if (refIter == reference.end())
    {
      niftkNiftyCalThrow() << "Reference data set does not contain point " << (*iter).first << std::endl;
    }

    double d2 = ((*iter).second.point.x - (*refIter).second.point.x)
              * ((*iter).second.point.x - (*refIter).second.point.x)
              + ((*iter).second.point.y - (*refIter).second.point.y)
              * ((*iter).second.point.y - (*refIter).second.point.y);

    queue.push(P(d2, (*iter).first));
  }

  unsigned int numberToKeep = queue.size() * percentage;
  unsigned int numberIncluded = 0;
  while (numberIncluded < numberToKeep)
  {
    P top = queue.top();
    iter = input.find(top.second);
    result.insert(*iter);
    queue.pop();
    numberIncluded++;
  }
  return result;
}


//-----------------------------------------------------------------------------
cv::Mat DrawEpiLines(const PointSet& leftDistortedPoints,
                     const cv::Mat&  leftIntrinsics,
                     const cv::Mat&  leftDistortion,
                     const int& whichImage,
                     const cv::Mat&  fundamentalMatrix,
                     const cv::Mat&  rightDistortedGreyImage,
                     const cv::Mat&  rightIntrinsics,
                     const cv::Mat&  rightDistortion
                    )
{
  cv::Mat undistortedRightImage;
  cv::undistort(rightDistortedGreyImage, undistortedRightImage, rightIntrinsics, rightDistortion, rightIntrinsics);

  PointSet undistortedLeftPoints;
  niftk::UndistortPoints(leftDistortedPoints, leftIntrinsics, leftDistortion, undistortedLeftPoints);

  std::vector<cv::Point2f> leftUndistP;
  std::vector<niftk::NiftyCalIdType> leftUndistId;
  niftk::ConvertPoints(undistortedLeftPoints, leftUndistP, leftUndistId);

  std::vector<cv::Point3f> epiLines;
  cv::computeCorrespondEpilines(leftUndistP, whichImage, fundamentalMatrix, epiLines);

  cv::Mat colouredUndistortedRightImage;
  cv::cvtColor(undistortedRightImage, colouredUndistortedRightImage, CV_GRAY2BGR);

  cv::RNG rng(0);
  for (size_t i = 0; i < epiLines.size(); i++)
  {
    cv::Scalar colour(rng(256),rng(256),rng(256));
    cv::line(colouredUndistortedRightImage,
             cv::Point(0,-epiLines[i].z/epiLines[i].y),
             cv::Point(colouredUndistortedRightImage.cols,
                       -(epiLines[i].z+epiLines[i].x*colouredUndistortedRightImage.cols)/epiLines[i].y),
             colour);
  }

  return colouredUndistortedRightImage;
}


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

  #pragma omp parallel private(u1), private(u2), private(u1t), private(u2t), private(u1p), private(u2p), private(r)
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


//-----------------------------------------------------------------------------
Model3D TransformModel(const Model3D& inputModel, const cv::Matx44d& matrix)
{
  Model3D output;
  cv::Matx41d p;
  cv::Matx41d q;
  Point3D op;

  Model3D::const_iterator iter;
  for (iter = inputModel.begin(); iter != inputModel.end(); ++iter)
  {
    p(0,0) = (*iter).second.point.x;
    p(1,0) = (*iter).second.point.y;
    p(2,0) = (*iter).second.point.z;
    p(3,0) = 1;

    q = matrix * p;
    op.point.x = q(0,0);
    op.point.y = q(1,0);
    op.point.z = q(2,0);
    op.id = (*iter).first;

    output.insert(IdPoint3D((*iter).first, op));
  }
  return output;
}


//-----------------------------------------------------------------------------
PointSet AddGaussianNoise(const PointSet& points,
                          const double& mean,
                          const double& stdDev
                         )
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, stdDev);

  PointSet output;
  Point2D op;

  PointSet::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter)
  {
    op.id = (*iter).first;
    op.point.x = (*iter).second.point.x + distribution(generator);
    op.point.y = (*iter).second.point.y + distribution(generator);
    output.insert(IdPoint2D((*iter).first, op));
  }

  return output;
}


//-----------------------------------------------------------------------------
double ComputeRMSReconstructionError(
  const Model3D& model,
  const std::list<PointSet>& listOfLeftHandPointSets,
  const std::list<PointSet>& listOfRightHandPointSets,
  const cv::Mat& intrinsicLeft,
  const cv::Mat& distortionLeft,
  const std::vector<cv::Mat>& rvecsLeft,
  const std::vector<cv::Mat>& tvecsLeft,
  const cv::Mat& intrinsicRight,
  const cv::Mat& distortionRight,
  const cv::Mat& leftToRightRotationMatrix,
  const cv::Mat& leftToRightTranslationVector,
  cv::Point3d& rmsForEachAxis
 )
{
  return 0;
}

} // end namespace
