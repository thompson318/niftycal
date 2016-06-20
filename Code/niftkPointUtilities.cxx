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
#include <queue>
#include <vector>
#include <functional>

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

} // end namespace
