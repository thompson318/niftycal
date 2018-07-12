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
#include "niftkMatrixUtilities.h"
#include <Internal/niftkTriangulationUtilities_p.h>
#include "niftkNiftyCalTypes.h"
#include <queue>
#include <vector>
#include <functional>

namespace niftk {

//-----------------------------------------------------------------------------
void DumpPoints(std::ostream& s, const PointSet& p)
{
  PointSet::const_iterator iter;
  for (iter = p.begin(); iter != p.end(); ++iter)
  {
    s << (*iter).second.id << " "
      << (*iter).second.point.x << " "
      << (*iter).second.point.y << std::endl;
  }
}


//-----------------------------------------------------------------------------
void DumpPoints(std::ostream& s, const Model3D& p)
{
  Model3D::const_iterator iter;
  for (iter = p.begin(); iter != p.end(); ++iter)
  {
    s << (*iter).second.id << " "
      << (*iter).second.point.x << " "
      << (*iter).second.point.y << " "
      << (*iter).second.point.z << " "
      << std::endl;
  }
}


//-----------------------------------------------------------------------------
double DistanceBetween(const cv::Point3d& a, const cv::Point3d& b)
{
  return sqrt(  (a.x - b.x) * (a.x - b.x)
              + (a.y - b.y) * (a.y - b.y)
              + (a.z - b.z) * (a.z - b.z)
             );
}


//-----------------------------------------------------------------------------
double DistanceBetween(const cv::Point2d& a, const cv::Point2d& b)
{
  return sqrt(  (a.x - b.x) * (a.x - b.x)
              + (a.y - b.y) * (a.y - b.y)
             );
}


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
bool PointSetContainsNonIntegerPositions(const PointSet& points)
{
  bool containsNonIntegerPoints = false;
  if (points.empty())
  {
    return containsNonIntegerPoints;
  }

  niftk::PointSet::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter)
  {
    niftk::Point2D p = (*iter).second;
    if (p.point.x - static_cast<int>(p.point.x) != 0)
    {
      containsNonIntegerPoints = true;
    }
    if (p.point.y - static_cast<int>(p.point.y) != 0)
    {
      containsNonIntegerPoints = true;
    }
  }
  return containsNonIntegerPoints;
}


//-----------------------------------------------------------------------------
bool MatchesToWithinTolerance(const PointSet& a, const PointSet& b, const double& tolerance)
{
  if (a.size() != b.size())
  {
    return false;
  }
  else
  {
    niftk::PointSet::const_iterator iter;
    for (iter = a.begin(); iter != a.end(); ++iter)
    {
      niftk::NiftyCalIdType id = (*iter).first;
      niftk::PointSet::const_iterator bIter = b.find(id);
      if (bIter == b.end())
      {
        return false;
      }
      cv::Point2d ap = (*iter).second.point;
      cv::Point2d bp = (*bIter).second.point;
      double distance = niftk::DistanceBetween(ap, bp);
      if (distance > tolerance)
      {
        return false;
      }
    }
  }
  return true;
}


//-----------------------------------------------------------------------------
void ExtractCommonPoints(const PointSet& inputA,
                         const PointSet& inputB,
                         std::vector<cv::Point2f>& outputA,
                         std::vector<cv::Point2f>& outputB,
                         std::vector<niftk::NiftyCalIdType>& commonIds
                         )
{
  outputA.clear();
  outputB.clear();
  commonIds.clear();

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

      commonIds.push_back((*iterA).first);
    }
  }
}


//-----------------------------------------------------------------------------
void ExtractCommonPoints(const Model3D& inputA,
                         const Model3D& inputB,
                         std::vector<cv::Point3d>& outputA,
                         std::vector<cv::Point3d>& outputB,
                         std::vector<niftk::NiftyCalIdType>& commonIds
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

      commonIds.push_back((*iterA).first);
    }
  }
}


//-----------------------------------------------------------------------------
bool ModelIsPlanar(const Model3D& model)
{
  bool result = true;

   Model3D::const_iterator iter;
   for (iter = model.begin(); iter != model.end(); ++iter)
   {
     if ((*iter).second.point.z != 0)
     {
       result = false;
     }
   }
   return result;
}


//-----------------------------------------------------------------------------
void ExtractCommonPoints(const Model3D& inputA,
                         const PointSet& inputB,
                         std::vector<cv::Point3d>& outputA,
                         std::vector<cv::Point2d>& outputB,
                         std::vector<niftk::NiftyCalIdType>& commonIds
                        )
{
  Model3D::const_iterator iterA;
  PointSet::const_iterator iterB;

  for (iterB = inputB.begin(); iterB != inputB.end(); ++iterB)
  {
    iterA = inputA.find(iterB->second.id);
    if (iterA != inputA.end())
    {
      cv::Point3d a;
      a.x = (*iterA).second.point.x;
      a.y = (*iterA).second.point.y;
      a.z = (*iterA).second.point.z;
      outputA.push_back(a);

      cv::Point2d b;
      b.x = (*iterB).second.point.x;
      b.y = (*iterB).second.point.y;
      outputB.push_back(b);

      commonIds.push_back((*iterA).first);
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
    niftkNiftyCalThrow() << "Different number of points ("
                         << inputPoint.size()
                         << ") and ids ("
                         << inputId.size()
                         << ").";
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
                                                 cv::Point2d& sumSquaredError,
                                                 cv::Point2d& rmsForEachAxis
                                                 )
{
  std::vector<cv::Point2f> a;
  std::vector<cv::Point2f> b;
  std::vector<niftk::NiftyCalIdType> commonIds;

  niftk::ExtractCommonPoints(inputA, inputB, a, b, commonIds);
  if (a.size() == 0 || b.size() == 0)
  {
    niftkNiftyCalThrow() << "No common points.";
  }
  if (a.size() != b.size())
  {
    niftkNiftyCalThrow() << "Programming errors, invalid extraction of common points.";
  }

  double rms = 0;
  sumSquaredError.x = 0;
  sumSquaredError.y = 0;
  rmsForEachAxis.x = 0;
  rmsForEachAxis.y = 0;
  cv::Point2d diff;
  cv::Point2d squaredDiff;

  for (size_t i = 0; i < a.size(); i++)
  {
    diff.x = a[i].x - b[i].x;
    diff.y = a[i].y - b[i].y;

    squaredDiff.x = diff.x * diff.x;
    squaredDiff.y = diff.y * diff.y;

    rms += squaredDiff.x + squaredDiff.y;

    rmsForEachAxis.x += squaredDiff.x;
    rmsForEachAxis.y += squaredDiff.y;

    sumSquaredError.x += squaredDiff.x;
    sumSquaredError.y += squaredDiff.y;
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
                                                 cv::Point3d& sumSquaredError,
                                                 cv::Point3d& rmsForEachAxis
                                                )
{
  std::vector<cv::Point3d> a;
  std::vector<cv::Point3d> b;
  std::vector<niftk::NiftyCalIdType> commonIds;

  niftk::ExtractCommonPoints(inputA, inputB, a, b, commonIds);
  if (a.size() == 0 || b.size() == 0)
  {
    niftkNiftyCalThrow() << "No common points.";
  }
  if (a.size() != b.size())
  {
    niftkNiftyCalThrow() << "Programming errors, invalid extraction of common points.";
  }

  double rms = 0;
  sumSquaredError.x = 0;
  sumSquaredError.y = 0;
  sumSquaredError.z = 0;
  rmsForEachAxis.x = 0;
  rmsForEachAxis.y = 0;
  rmsForEachAxis.z = 0;
  cv::Point3d diff;
  cv::Point3d squaredDiff;

  for (size_t i = 0; i < a.size(); i++)
  {
    diff.x = (a[i].x - b[i].x);
    diff.y = (a[i].y - b[i].y);
    diff.z = (a[i].z - b[i].z);

    squaredDiff.x = diff.x * diff.x;
    squaredDiff.y = diff.y * diff.y;
    squaredDiff.z = diff.z * diff.z;

    rms += squaredDiff.x + squaredDiff.y + squaredDiff.z;

    rmsForEachAxis.x += squaredDiff.x;
    rmsForEachAxis.y += squaredDiff.y;
    rmsForEachAxis.z += squaredDiff.z;

    sumSquaredError.x += squaredDiff.x;
    sumSquaredError.y += squaredDiff.y;
    sumSquaredError.z += squaredDiff.z;
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
PointSet AddGaussianNoise(std::default_random_engine& engine,
                          std::normal_distribution<double>& normalDistribution,
                          const PointSet& points
                         )
{
  PointSet output;
  Point2D op;

  PointSet::const_iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter)
  {
    op.id = (*iter).first;
    op.point.x = (*iter).second.point.x + normalDistribution(engine);
    op.point.y = (*iter).second.point.y + normalDistribution(engine);
    output.insert(IdPoint2D((*iter).first, op));
  }

  return output;
}


//-----------------------------------------------------------------------------
cv::Mat ProjectPointsToImage(const Model3D& model,
                             const PointSet& points,
                             const cv::Matx44d& extrinsic,
                             const cv::Mat& intrinsic,
                             const cv::Mat& distortion,
                             const cv::Size& imageSize
                            )
{
  std::vector<cv::Point2f> observed;
  std::vector<cv::Point2f> projected;
  std::vector<niftk::NiftyCalIdType> ids;

  ProjectMatchingPoints(model, points, extrinsic, intrinsic, distortion, observed, projected, ids);
  cv::Mat image = cv::Mat::zeros(imageSize.height, imageSize.width, CV_8UC1);

  for (std::vector<cv::Point2f>::size_type i = 0; i < projected.size(); i++)
  {
    if (projected[i].x >= 0 && projected[i].y >= 0
        && projected[i].x <= imageSize.width - 1
        && projected[i].y <= imageSize.height - 1
        )
    {
      image.at<unsigned char>(projected[i].y, projected[i].x) = 255;
    }

  }
  return image;
}


//-----------------------------------------------------------------------------
unsigned int ProjectMatchingPoints(const Model3D& model,
                                   const PointSet& points,
                                   const cv::Matx44d& extrinsic,
                                   const cv::Mat& intrinsic,
                                   const cv::Mat& distortion,
                                   std::vector<cv::Point2f>& observed,
                                   std::vector<cv::Point2f>& projected,
                                   std::vector<niftk::NiftyCalIdType>& ids
                                  )
{
  cv::Point3d modelPoint;
  cv::Point3f m;
  cv::Point2f p;
  std::vector<cv::Point3f> modelPoints;
  niftk::PointSet::const_iterator pointIter;
  Model3D::const_iterator modelIter;
  unsigned int pointPerViewCounter = 0;

  cv::Mat extrinsicRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat extrinsicTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(extrinsic, extrinsicRotationVector, extrinsicTranslationVector);

  observed.clear();
  projected.clear();
  ids.clear();

  modelPoints.resize(points.size());
  projected.resize(points.size());
  observed.resize(points.size());
  ids.resize(points.size());

  for (pointIter = points.begin();
       pointIter != points.end();
       ++pointIter
       )
  {
    NiftyCalIdType id = (*pointIter).first;
    modelIter = model.find(id);
    if (modelIter == model.end())
    {
      niftkNiftyCalThrow() << "Invalid point id:" << id;
    }

    modelPoint = (*modelIter).second.point;
    m.x = modelPoint.x;
    m.y = modelPoint.y;
    m.z = modelPoint.z;
    modelPoints[pointPerViewCounter] = m;

    p.x = (*pointIter).second.point.x;
    p.y = (*pointIter).second.point.y;
    observed[pointPerViewCounter] = p;

    ids[pointPerViewCounter] = id;

    pointPerViewCounter++;
  }

  cv::projectPoints(modelPoints,
                    extrinsicRotationVector,
                    extrinsicTranslationVector,
                    intrinsic,
                    distortion,
                    projected);

  return pointPerViewCounter;
}


//-----------------------------------------------------------------------------
void TriangulatePointPairs(
  const PointSet& leftDistortedPoints,
  const PointSet& rightDistortedPoints,
  const cv::Mat& leftIntrinsics,
  const cv::Mat& leftDistortionParams,
  const cv::Mat& leftCameraRotationVector,
  const cv::Mat& leftCameraTranslationVector,
  const cv::Mat& leftToRightRotationMatrix,
  const cv::Mat& leftToRightTranslationVector,
  const cv::Mat& rightIntrinsics,
  const cv::Mat& rightDistortionParams,
  Model3D& outputTriangulatedPoints
  )
{
  PointSet leftUndistortedPoints;
  niftk::UndistortPoints(leftDistortedPoints, leftIntrinsics, leftDistortionParams, leftUndistortedPoints);

  PointSet rightUndistortedPoints;
  niftk::UndistortPoints(rightDistortedPoints, rightIntrinsics, rightDistortionParams, rightUndistortedPoints);

  std::vector<cv::Point2f> leftPoints;
  std::vector<cv::Point2f> rightPoints;
  std::vector<niftk::NiftyCalIdType> commonIds;
  niftk::ExtractCommonPoints(leftUndistortedPoints, rightUndistortedPoints, leftPoints, rightPoints, commonIds);

  cv::Matx44d leftExtrinsic =
      niftk::RodriguesToMatrix(leftCameraRotationVector, leftCameraTranslationVector);

  cv::Matx44d leftToRight =
      niftk::RotationAndTranslationToMatrix(leftToRightRotationMatrix, leftToRightTranslationVector);

  cv::Matx44d rightExtrinsic = leftToRight * leftExtrinsic;

  cv::Mat rightCameraRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat rightCameraTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(rightExtrinsic, rightCameraRotationVector, rightCameraTranslationVector);

  std::vector<cv::Point3f> triangulatedPoints;

  niftk::TriangulatePointPairs(leftPoints,
                               rightPoints,
                               leftIntrinsics,
                               leftCameraRotationVector,
                               leftCameraTranslationVector,
                               rightIntrinsics,
                               rightCameraRotationVector,
                               rightCameraTranslationVector,
                               triangulatedPoints
                               );

  outputTriangulatedPoints.clear();
  for (int i = 0; i < triangulatedPoints.size(); i++)
  {
    Point3D p;
    p.id = commonIds[i];
    p.point = triangulatedPoints[i];
    outputTriangulatedPoints.insert(niftk::IdPoint3D(commonIds[i], p));
  }
}


//-----------------------------------------------------------------------------
double ComputeRMSReconstructionError(const Model3D& model,
                                     const std::list<PointSet>& listOfLeftHandPointSets,
                                     const std::list<PointSet>& listOfRightHandPointSets,
                                     const cv::Mat& leftIntrinsics,
                                     const cv::Mat& leftDistortionParams,
                                     const std::vector<cv::Mat>& rvecsLeft,
                                     const std::vector<cv::Mat>& tvecsLeft,
                                     const cv::Mat& rightIntrinsics,
                                     const cv::Mat& rightDistortionParams,
                                     const cv::Mat& leftToRightRotationMatrix,
                                     const cv::Mat& leftToRightTranslationVector,
                                     cv::Point3d& rmsForEachAxis
                                    )
{
  double rms = 0;
  unsigned int pointCounter = 0;
  unsigned int viewCounter = 0;
  rmsForEachAxis.x = 0;
  rmsForEachAxis.y = 0;
  rmsForEachAxis.z = 0;

  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left and right points.";
  }
  if (listOfLeftHandPointSets.size() != rvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and rotation vectors.";
  }
  if (listOfLeftHandPointSets.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and translation vectors.";
  }

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;

  for (leftIter = listOfLeftHandPointSets.begin(),
       rightIter = listOfRightHandPointSets.begin();
       leftIter != listOfLeftHandPointSets.end() &&
       rightIter != listOfRightHandPointSets.end();
       ++leftIter,
       ++rightIter
       )
  {
    cv::Matx44d modelToCamera = niftk::RodriguesToMatrix(rvecsLeft[viewCounter], tvecsLeft[viewCounter]);
    Model3D modelInLeftCameraCoordinates = niftk::TransformModel(model, modelToCamera);

    Model3D triangulatedPoints;

    niftk::TriangulatePointPairs(
      *leftIter,
      *rightIter,
      leftIntrinsics,
      leftDistortionParams,
      rvecsLeft[viewCounter],
      tvecsLeft[viewCounter],
      leftToRightRotationMatrix,
      leftToRightTranslationVector,
      rightIntrinsics,
      rightDistortionParams,
      triangulatedPoints
    );

    cv::Point3d cameraViewRMS;
    cv::Point3d cameraViewSSE;

    double viewRMS = niftk::ComputeRMSDifferenceBetweenMatchingPoints(modelInLeftCameraCoordinates,
                                                                      triangulatedPoints,
                                                                      cameraViewSSE,
                                                                      cameraViewRMS
                                                                     );

    pointCounter += (*leftIter).size();
    rms += (viewRMS*viewRMS*static_cast<double>((*leftIter).size()));
    rmsForEachAxis.x += cameraViewSSE.x;
    rmsForEachAxis.y += cameraViewSSE.y;
    rmsForEachAxis.z += cameraViewSSE.z;

    viewCounter++;
  }

  if (pointCounter != 0)
  {
    rms /= static_cast<double>(pointCounter);
    rmsForEachAxis.x /= static_cast<double>(pointCounter);
    rmsForEachAxis.y /= static_cast<double>(pointCounter);
    rmsForEachAxis.z /= static_cast<double>(pointCounter);
  }

  rms = sqrt(rms);
  rmsForEachAxis.x = sqrt(rmsForEachAxis.x);
  rmsForEachAxis.y = sqrt(rmsForEachAxis.y);
  rmsForEachAxis.z = sqrt(rmsForEachAxis.z);

  return rms;
}


//-----------------------------------------------------------------------------
double ComputeRMSReprojectionError(
    const Model3D& model,
    const std::list<PointSet>& listOfLeftHandPointSets,
    const std::list<PointSet>& listOfRightHandPointSets,
    const cv::Mat& leftIntrinsics,
    const cv::Mat& leftDistortionParams,
    const std::vector<cv::Mat>& rvecsLeft,
    const std::vector<cv::Mat>& tvecsLeft,
    const cv::Mat& rightIntrinsics,
    const cv::Mat& rightDistortionParams,
    const cv::Mat& leftToRightRotationMatrix,
    const cv::Mat& leftToRightTranslationVector
    )
{
  double rms = 0;
  unsigned int pointCounter = 0;
  unsigned int pointCounterLeft = 0;
  unsigned int pointCounterRight = 0;
  unsigned int viewCounter = 0;

  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left and right points.";
  }
  if (listOfLeftHandPointSets.size() != rvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and rotation vectors.";
  }
  if (listOfLeftHandPointSets.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and translation vectors.";
  }

  cv::Matx44d leftToRight = niftk::RotationAndTranslationToMatrix(leftToRightRotationMatrix,
                                                                  leftToRightTranslationVector
                                                                 );

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;

  for (leftIter = listOfLeftHandPointSets.begin(),
       rightIter = listOfRightHandPointSets.begin();
       leftIter != listOfLeftHandPointSets.end() &&
       rightIter != listOfRightHandPointSets.end();
       ++leftIter,
       ++rightIter
       )
  {

    std::vector<cv::Point2f> observed;
    std::vector<cv::Point2f> projected;
    std::vector<niftk::NiftyCalIdType> ids;

    cv::Matx44d leftExtrinsic = niftk::RodriguesToMatrix(rvecsLeft[viewCounter], tvecsLeft[viewCounter]);
    unsigned int numberOfPointsInLeft = niftk::ProjectMatchingPoints(model,
                                                                     *leftIter,
                                                                     leftExtrinsic,
                                                                     leftIntrinsics,
                                                                     leftDistortionParams,
                                                                     observed,
                                                                     projected,
                                                                     ids
                                                                     );

    for (unsigned int i = 0; i < numberOfPointsInLeft; i++)
    {
      double dx = ((observed[i].x - projected[i].x) * (observed[i].x - projected[i].x));
      double dy = ((observed[i].y - projected[i].y) * (observed[i].y - projected[i].y));

      rms += dx;
      rms += dy;
    }

    cv::Matx44d rightExtrinsic = leftToRight * leftExtrinsic;
    unsigned int numberOfPointsInRight = niftk::ProjectMatchingPoints(model,
                                                                      *rightIter,
                                                                      rightExtrinsic,
                                                                      rightIntrinsics,
                                                                      rightDistortionParams,
                                                                      observed,
                                                                      projected,
                                                                      ids
                                                                      );
    for (unsigned int i = 0; i < numberOfPointsInRight; i++)
    {
      double dx = ((observed[i].x - projected[i].x) * (observed[i].x - projected[i].x));
      double dy = ((observed[i].y - projected[i].y) * (observed[i].y - projected[i].y));
      rms += dx;
      rms += dy;
    }

    pointCounter += (2 * (numberOfPointsInLeft + numberOfPointsInRight));
    pointCounterLeft += 2*numberOfPointsInLeft;
    pointCounterRight += 2*numberOfPointsInRight;
    viewCounter++;
  }

  if (pointCounter != 0)
  {
    rms /= static_cast<double>(pointCounter);
  }

  rms = sqrt(rms);
  return rms;
}


//-----------------------------------------------------------------------------
double ComputeRMSReconstructionError(const Model3D& model,
                                     const std::list<PointSet>& listOfLeftHandPointSets,
                                     const std::list<PointSet>& listOfRightHandPointSets,
                                     const cv::Mat& leftIntrinsics,
                                     const cv::Mat& leftDistortionParams,
                                     const std::vector<cv::Mat>& rvecsLeft,
                                     const std::vector<cv::Mat>& tvecsLeft,
                                     const cv::Mat& rightIntrinsics,
                                     const cv::Mat& rightDistortionParams,
                                     const cv::Mat& leftToRightRotationMatrix,
                                     const cv::Mat& leftToRightTranslationVector,
                                     const std::list<cv::Matx44d>& trackingMatrices,
                                     const cv::Matx44d& handEyeMatrix,
                                     const cv::Matx44d& modelToWorldMatrix
                                    )
{
  double rms = 0;
  unsigned int pointCounter = 0;
  unsigned int viewCounter = 0;

  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left and right points.";
  }
  if (listOfLeftHandPointSets.size() != trackingMatrices.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and tracking matrices.";
  }
  if (listOfLeftHandPointSets.size() != rvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and rotation vectors.";
  }
  if (listOfLeftHandPointSets.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left points and translation vectors.";
  }

  cv::Matx44d eyeHandMatrix = handEyeMatrix.inv(cv::DECOMP_SVD);
  cv::Matx44d worldToModel = modelToWorldMatrix.inv(cv::DECOMP_SVD);

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;
  std::list<cv::Matx44d>::const_iterator matrixIter;

  for (leftIter = listOfLeftHandPointSets.begin(),
       rightIter = listOfRightHandPointSets.begin(),
       matrixIter = trackingMatrices.begin();
       leftIter != listOfLeftHandPointSets.end() &&
       rightIter != listOfRightHandPointSets.end() &&
       matrixIter != trackingMatrices.end();
       ++leftIter,
       ++rightIter,
       ++matrixIter
       )
  {
    Model3D triangulatedPoints;

    niftk::TriangulatePointPairs(
      *leftIter,
      *rightIter,
      leftIntrinsics,
      leftDistortionParams,
      rvecsLeft[viewCounter],
      tvecsLeft[viewCounter],
      leftToRightRotationMatrix,
      leftToRightTranslationVector,
      rightIntrinsics,
      rightDistortionParams,
      triangulatedPoints
    );


    cv::Matx44d cameraToModel = worldToModel * (*matrixIter) * eyeHandMatrix;
    Model3D transformedPoints = niftk::TransformModel(triangulatedPoints, cameraToModel);

    cv::Point3d cameraViewRMS;
    cv::Point3d cameraViewSSE;

    double viewRMS = niftk::ComputeRMSDifferenceBetweenMatchingPoints(model,
                                                                      transformedPoints,
                                                                      cameraViewSSE,
                                                                      cameraViewRMS
                                                                     );

    pointCounter += (*leftIter).size();
    rms += (viewRMS*viewRMS*static_cast<double>((*leftIter).size()));

    viewCounter++;
  }

  if (pointCounter != 0)
  {
    rms /= static_cast<double>(pointCounter);
  }

  rms = sqrt(rms);

  viewCounter++;
  return rms;
}


//-----------------------------------------------------------------------------
double ComputeRMSReconstructionError(const Model3D& model,
                                     const std::list<PointSet>& pointSets,
                                     const std::vector<cv::Mat>& rvecs,
                                     const std::vector<cv::Mat>& tvecs,
                                     const std::list<cv::Matx44d>& trackingMatrices,
                                     const cv::Matx44d& handEyeMatrix,
                                     const cv::Matx44d& modelToWorldMatrix
                                    )
{
  double rms = 0;
  unsigned int pointCounter = 0;
  unsigned int viewCounter = 0;

  if (pointSets.size() != trackingMatrices.size())
  {
    niftkNiftyCalThrow() << "Unequal number of points and tracking matrices.";
  }
  if (pointSets.size() != rvecs.size())
  {
    niftkNiftyCalThrow() << "Unequal number of points and rotation vectors.";
  }
  if (pointSets.size() != tvecs.size())
  {
    niftkNiftyCalThrow() << "Unequal number of points and translation vectors.";
  }

  cv::Matx44d eyeHandMatrix = handEyeMatrix.inv(cv::DECOMP_SVD);
  cv::Matx44d worldToModel = modelToWorldMatrix.inv(cv::DECOMP_SVD);

  std::list<PointSet>::const_iterator pointSetIter;
  std::list<cv::Matx44d>::const_iterator matrixIter;

  for (pointSetIter = pointSets.begin(),
       matrixIter = trackingMatrices.begin();
       pointSetIter != pointSets.end() &&
       matrixIter != trackingMatrices.end();
       ++pointSetIter,
       ++matrixIter
       )
  {
    cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvecs[viewCounter], tvecs[viewCounter]);
    cv::Matx44d transform = worldToModel * (*matrixIter) * eyeHandMatrix * extrinsic;

    Model3D transformedPoints = niftk::TransformModel(model, transform);

    cv::Point3d cameraViewRMS;
    cv::Point3d cameraViewSSE;

    double viewRMS = niftk::ComputeRMSDifferenceBetweenMatchingPoints(model,
                                                                      transformedPoints,
                                                                      cameraViewSSE,
                                                                      cameraViewRMS
                                                                     );

    pointCounter += (*pointSetIter).size();
    rms += (viewRMS*viewRMS*static_cast<double>((*pointSetIter).size()));

    viewCounter++;
  }

  if (pointCounter != 0)
  {
    rms /= static_cast<double>(pointCounter);
  }

  rms = sqrt(rms);

  viewCounter++;
  return rms;
}


//-----------------------------------------------------------------------------
unsigned long int GetNumberOfTriangulatablePoints(const Model3D& model,
                                                  const std::list<PointSet>& leftPoints,
                                                  const std::list<PointSet>& rightPoints
                                                 )
{
  if (leftPoints.size() != rightPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left and right points.";
  }

  unsigned long int numberOfTriangulatablePoints = 0;
  std::list<PointSet>::const_iterator leftViewIter;
  std::list<PointSet>::const_iterator rightViewIter;
  niftk::PointSet::const_iterator pointIter;

  for (leftViewIter = leftPoints.begin(),
       rightViewIter = rightPoints.begin();
       leftViewIter != leftPoints.end() && rightViewIter != rightPoints.end();
       ++leftViewIter,
       ++rightViewIter
       )
  {
    for (pointIter = leftViewIter->begin();
         pointIter != leftViewIter->end();
         ++pointIter
         )
    {
      niftk::NiftyCalIdType id = (*pointIter).first;
      if (rightViewIter->find(id) != rightViewIter->end()
          && model.find(id) != model.end()
          )
      {
        numberOfTriangulatablePoints++;
      }
    }
  }
  return numberOfTriangulatablePoints;
}


//-----------------------------------------------------------------------------
double ComputeRMSReprojectionError(const Model3D& model,
                                   const PointSet& imagePoints,
                                   const cv::Mat& intrinsic,
                                   const cv::Mat& distortion,
                                   const cv::Mat& rvec,
                                   const cv::Mat& tvec
                                  )
{
  cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

  std::vector<cv::Point2f> observed;
  std::vector<cv::Point2f> projected;
  std::vector<niftk::NiftyCalIdType> ids;
  niftk::ProjectMatchingPoints(model, imagePoints, extrinsic, intrinsic, distortion, observed, projected, ids);

  double rms = 0;
  for (unsigned int i = 0; i < observed.size(); i++)
  {
    rms += ((observed[i].x - projected[i].x) * (observed[i].x - projected[i].x)
           + (observed[i].y - projected[i].y) * (observed[i].y - projected[i].y));
  }
  if (observed.size() > 0)
  {
    rms /= static_cast<double>(observed.size());
  }
  rms = sqrt(rms);

  return rms;
}


//-----------------------------------------------------------------------------
cv::Point3d GetCentroid(const std::vector<cv::Point3d>& points)
{
  double numberOfPoints = static_cast<double>(points.size());
  if (numberOfPoints < 1)
  {
    niftkNiftyCalThrow() << "The number of points should be >= 1";
  }

  cv::Point3d centroid(0, 0, 0);

  for (std::vector<cv::Point3d>::size_type i = 0; i < numberOfPoints; ++i)
  {
    centroid += points[i];
  }
  centroid.x /= numberOfPoints;
  centroid.y /= numberOfPoints;
  centroid.z /= numberOfPoints;

  return centroid;
}


//-----------------------------------------------------------------------------
std::vector<cv::Point3d> SubtractCentroid(const std::vector<cv::Point3d>& points,
                                          const cv::Point3d& centroid)
{
  std::vector<cv::Point3d> result;

  for (std::vector<cv::Point3d>::size_type i = 0; i < points.size(); ++i)
  {
    cv::Point3d tmp = points[i] - centroid;
    result.push_back(tmp);
  }
  return result;
}


//-----------------------------------------------------------------------------
double CalculateFiducialRegistrationError(const std::vector<cv::Point3d>& fixedPoints,
                                          const std::vector<cv::Point3d>& movingPoints,
                                          const cv::Matx44d& matrix
                                         )
{
  if (fixedPoints.size() != movingPoints.size())
  {
    niftkNiftyCalThrow() << "The number of 'fixed' points is " << fixedPoints.size()
                         << " whereas the number of 'moving' points is " << movingPoints.size()
                         << " and they should correspond.";
  }

  double fre = 0;
  std::vector<cv::Point3d>::size_type numberOfPoints = fixedPoints.size();

  for (std::vector<cv::Point3d>::size_type i = 0; i < numberOfPoints; ++i)
  {
    cv::Matx41d f, m, mPrime;
    f(0,0) = fixedPoints[i].x;
    f(1,0) = fixedPoints[i].y;
    f(2,0) = fixedPoints[i].z;
    f(3,0) = 1;
    m(0,0) = movingPoints[i].x;
    m(1,0) = movingPoints[i].y;
    m(2,0) = movingPoints[i].z;
    m(3,0) = 1;
    mPrime = matrix * m;
    double squaredError =   (f(0,0) - mPrime(0,0)) * (f(0,0) - mPrime(0,0))
                          + (f(1,0) - mPrime(1,0)) * (f(1,0) - mPrime(1,0))
                          + (f(2,0) - mPrime(2,0)) * (f(2,0) - mPrime(2,0))
                          ;
    fre += squaredError;
  }
  if (numberOfPoints > 0)
  {
    fre /= (double)numberOfPoints;
  }
  fre = std::sqrt(fre);
  return fre;
}


//-----------------------------------------------------------------------------
double RegisterPoints(const std::vector<cv::Point3d>& fixedPoints,
                      const std::vector<cv::Point3d>& movingPoints,
                      cv::Matx44d& rigidBodyMatrix
                     )
{
  if (fixedPoints.size() < 3)
  {
    niftkNiftyCalThrow() << "The number of 'fixed' points is < 3";
  }

  if (movingPoints.size() < 3)
  {
    niftkNiftyCalThrow() << "The number of 'moving' points is < 3";
  }

  if (fixedPoints.size() != movingPoints.size())
  {
    niftkNiftyCalThrow() << "The number of 'fixed' points is " << fixedPoints.size()
                         << " whereas the number of 'moving' points is " << movingPoints.size()
                         << " and they should match.";
  }

  // Based on Arun's method:
  // Least-Squares Fitting of two, 3-D Point Sets, Arun, 1987,
  // 10.1109/TPAMI.1987.4767965
  //
  // Also See:
  // http://eecs.vanderbilt.edu/people/mikefitzpatrick/papers/ (new line)
  // 2009_Medim_Fitzpatrick_TRE_FRE_uncorrelated_as_published.pdf
  // Then:
  // http://tango.andrew.cmu.edu/~gustavor/42431-intro-bioimaging/readings/ch8.pdf

  // Arun Equation 4.
  cv::Point3d pPrime = GetCentroid(fixedPoints);

  // Arun Equation 6.
  cv::Point3d p = GetCentroid(movingPoints);

  // Arun Equation 7.
  std::vector<cv::Point3d> q = SubtractCentroid(movingPoints, p);

  // Arun Equation 8.
  std::vector<cv::Point3d> qPrime = SubtractCentroid(fixedPoints, pPrime);

  // Arun Equation 11.
  cv::Matx33d H = cv::Matx33d::zeros();
  for (std::vector<cv::Point3d>::size_type i = 0; i < q.size(); ++i)
  {
    cv::Matx33d tmp(
          q[i].x*qPrime[i].x, q[i].x*qPrime[i].y, q[i].x*qPrime[i].z,
          q[i].y*qPrime[i].x, q[i].y*qPrime[i].y, q[i].y*qPrime[i].z,
          q[i].z*qPrime[i].x, q[i].z*qPrime[i].y, q[i].z*qPrime[i].z
        );
    H += tmp;
  }

  // Arun Equation 12.
  cv::SVD svd(H);

  // Arun Equation 13.
  // cv::Mat X = svd.vt.t() * svd.u.t();

  // Replace Arun Equation 13 with Fitzpatrick, chapter 8, page 470.
  cv::Mat VU = svd.vt.t() * svd.u;
  double detVU = cv::determinant(VU);
  cv::Matx33d diag = cv::Matx33d::zeros();
  diag(0,0) = 1;
  diag(1,1) = 1;
  diag(2,2) = detVU;
  cv::Mat diagonal(diag);
  cv::Mat X = (svd.vt.t() * (diagonal * svd.u.t()));

  // Arun Step 5.
  double detX = cv::determinant(X);

  if ( detX < 0
       && (   std::fabs(svd.w.at<double>(0,0)) < 0.000001
           || std::fabs(svd.w.at<double>(1,1)) < 0.000001
           || std::fabs(svd.w.at<double>(2,2)) < 0.000001
          )
     )
  {
    // Implement 2a in section VI in Arun paper.
    cv::Mat VPrime = svd.vt.t();
    VPrime.at<double>(0,2) = -1.0 * VPrime.at<double>(0,2);
    VPrime.at<double>(1,2) = -1.0 * VPrime.at<double>(1,2);
    VPrime.at<double>(2,2) = -1.0 * VPrime.at<double>(2,2);
    X = VPrime * svd.u.t();
    detX = cv::determinant(X);
  }

  if (detX < 0)
  {
    niftkNiftyCalThrow() << "Determinant < 0";
  }

  // Arun Equation 10.
  cv::Matx31d T, tmpP, tmpPPrime;
  cv::Matx33d R(X);
  tmpP(0,0) = p.x;
  tmpP(1,0) = p.y;
  tmpP(2,0) = p.z;
  tmpPPrime(0,0) = pPrime.x;
  tmpPPrime(1,0) = pPrime.y;
  tmpPPrime(2,0) = pPrime.z;
  T = tmpPPrime - R*tmpP;

  rigidBodyMatrix = cv::Matx44d::eye();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      rigidBodyMatrix(i, j) = R(i, j);
    }
    rigidBodyMatrix(i, 3) = T(i, 0);
  }

  double fre = CalculateFiducialRegistrationError(fixedPoints, movingPoints, rigidBodyMatrix);
  return fre;
}


//-----------------------------------------------------------------------------
double ComputeLeftToRight(const Model3D& model,
                          const std::vector<cv::Mat>& rvecsLeft,
                          const std::vector<cv::Mat>& tvecsLeft,
                          const std::vector<cv::Mat>& rvecsRight,
                          const std::vector<cv::Mat>& tvecsRight,
                          cv::Mat& leftToRightRotationMatrix,
                          cv::Mat& leftToRightTranslationVector
                         )
{
  if (rvecsLeft.size() < 1)
  {
    niftkNiftyCalThrow() << "The number of 'rvecsLeft' is < 1.";
  }
  if (rvecsLeft.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "The number of 'rvecsLeft' doesn't match the number of 'tvecsLeft'." << std::endl;
  }
  if (rvecsLeft.size() != rvecsRight.size())
  {
    niftkNiftyCalThrow() << "The number of 'rvecsLeft' doesn't match the number of 'rvecsRight'." << std::endl;
  }
  if (rvecsLeft.size() != tvecsRight.size())
  {
    niftkNiftyCalThrow() << "The number of 'rvecsLeft' doesn't match the number of 'rvecsRight'." << std::endl;
  }
  if (model.size() < 3)
  {
    niftkNiftyCalThrow() << "The model must have at least 3 points.";
  }

  std::vector<cv::Point3d> fixedPoints;
  std::vector<cv::Point3d> movingPoints;

  for (std::vector<cv::Mat>::size_type i = 0; i < rvecsLeft.size(); i++)
  {
    cv::Matx44d leftExtrinsics = RodriguesToMatrix(rvecsLeft[i], tvecsLeft[i]);
    cv::Matx44d rightExtrinsics = RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

    cv::Point3d tmpModelPoint;
    cv::Point3d tmpLeftPoint;
    cv::Point3d tmpRightPoint;

    cv::Matx41d modelPoint;
    cv::Matx41d transformedModelPointLeft;
    cv::Matx41d transformedModelPointRight;

    for (Model3D::const_iterator iter = model.begin(); iter != model.end(); ++iter)
    {
      tmpModelPoint = (*iter).second.point;
      modelPoint(0, 0) = tmpModelPoint.x;
      modelPoint(1, 0) = tmpModelPoint.y;
      modelPoint(2, 0) = tmpModelPoint.z;
      modelPoint(3, 0) = 1;

      transformedModelPointLeft = leftExtrinsics * modelPoint;
      transformedModelPointRight = rightExtrinsics * modelPoint;

      tmpLeftPoint.x = transformedModelPointLeft(0, 0);
      tmpLeftPoint.y = transformedModelPointLeft(1, 0);
      tmpLeftPoint.z = transformedModelPointLeft(2, 0);

      tmpRightPoint.x = transformedModelPointRight(0, 0);
      tmpRightPoint.y = transformedModelPointRight(1, 0);
      tmpRightPoint.z = transformedModelPointRight(2, 0);

      movingPoints.push_back(tmpLeftPoint);
      fixedPoints.push_back(tmpRightPoint);
    }
  }
  cv::Matx44d registrationMatrix;

  double fre = RegisterPoints(fixedPoints, movingPoints, registrationMatrix);

  cv::Mat rotationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(registrationMatrix, rotationVector, leftToRightTranslationVector);
  cv::Rodrigues(rotationVector, leftToRightRotationMatrix);

  return fre;
}


//-----------------------------------------------------------------------------
bool IsCrossEyed(const cv::Mat& intrinsicLeft,
                 const cv::Mat& distortionLeft,
                 const cv::Mat& rvecLeft,
                 const cv::Mat& tvecLeft,
                 const cv::Mat& intrinsicRight,
                 const cv::Mat& distortionRight,
                 const cv::Mat& rvecRight,
                 const cv::Mat& tvecRight,
                 cv::Point3d* convergencePoint,
                 const double& maximumUsableDistance
                )
{
  bool isCrossEyed = false;

  cv::Mat leftToRightRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
  cv::Mat leftToRightTranslationVector = cvCreateMat(3, 1, CV_64FC1);
  niftk::GetLeftToRightMatrix(rvecLeft,
                              tvecLeft,
                              rvecRight,
                              tvecRight,
                              leftToRightRotationMatrix,
                              leftToRightTranslationVector);

  niftk::Point2D p;
  p.id = 0;

  p.point.x = intrinsicLeft.at<double>(0, 2);
  p.point.y = intrinsicLeft.at<double>(1, 2);

  niftk::PointSet leftPrincipalPoint;
  leftPrincipalPoint.insert(std::pair<NiftyCalIdType, Point2D>(0, p));

  p.point.x = intrinsicRight.at<double>(0, 2);
  p.point.y = intrinsicRight.at<double>(1, 2);

  niftk::PointSet rightPrincipalPoint;
  rightPrincipalPoint.insert(std::pair<NiftyCalIdType, Point2D>(0, p));

  niftk::Model3D triangulatedPoints;
  niftk::TriangulatePointPairs(leftPrincipalPoint,
                               rightPrincipalPoint,
                               intrinsicLeft,
                               distortionLeft,
                               rvecLeft,
                               tvecLeft,
                               leftToRightRotationMatrix,
                               leftToRightTranslationVector,
                               intrinsicRight,
                               distortionRight,
                               triangulatedPoints
                              );

  if (triangulatedPoints.size() != 1)
  {
    niftkNiftyCalThrow() << "There should be exactly 1 triangulated vergence point.";
  }

  cv::Point3d tp = (*(triangulatedPoints.begin())).second.point;

  if (convergencePoint != nullptr)
  {
    *convergencePoint = tp;
  }

  if (tp.z > 0 && tp.z < maximumUsableDistance)
  {
    isCrossEyed = true;
  }

  return isCrossEyed;
}


//-----------------------------------------------------------------------------
void CheckAgainstConvergencePoint(const std::list<PointSet>& leftDistortedPoints,
                                  const std::list<PointSet>& rightDistortedPoints,
                                  const cv::Mat& intrinsicLeft,
                                  const cv::Mat& distortionLeft,
                                  const std::vector<cv::Mat>& rvecsLeft,
                                  const std::vector<cv::Mat>& tvecsLeft,
                                  const cv::Mat& intrinsicRight,
                                  const cv::Mat& distortionRight,
                                  const std::vector<cv::Mat>& rvecsRight,
                                  const std::vector<cv::Mat>& tvecsRight,
                                  const cv::Point3d& convergencePoint,
                                  bool& somePointsAreNearer,
                                  bool& somePointsAreFurther
                                 )
{
  somePointsAreNearer = false;
  somePointsAreFurther = false;

  double depth = convergencePoint.z;

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;
  unsigned int viewCounter = 0;
  for (leftIter = leftDistortedPoints.begin(),
       rightIter = rightDistortedPoints.begin();
       leftIter != leftDistortedPoints.end() &&
       rightIter != rightDistortedPoints.end();
       leftIter++,
       rightIter++
       )
  {
    cv::Mat leftToRightRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
    cv::Mat leftToRightTranslationVector = cvCreateMat(3, 1, CV_64FC1);
    niftk::GetLeftToRightMatrix(rvecsLeft[viewCounter],
                                tvecsLeft[viewCounter],
                                rvecsRight[viewCounter],
                                tvecsRight[viewCounter],
                                leftToRightRotationMatrix,
                                leftToRightTranslationVector);

    niftk::Model3D triangulatedPoints;
    niftk::TriangulatePointPairs(*leftIter,
                                 *rightIter,
                                 intrinsicLeft,
                                 distortionLeft,
                                 rvecsLeft[viewCounter],
                                 tvecsLeft[viewCounter],
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 intrinsicRight,
                                 distortionRight,
                                 triangulatedPoints
                                );

    niftk::Model3D::const_iterator modelIter;
    for (modelIter = triangulatedPoints.begin();
         modelIter != triangulatedPoints.end();
         modelIter++
        )
    {
      if (modelIter->second.point.z > depth)
      {
        somePointsAreFurther = true;
      }
      if (modelIter->second.point.z <= depth)
      {
        somePointsAreNearer = true;
      }
    }
    viewCounter++;
  }
}

} // end namespace
