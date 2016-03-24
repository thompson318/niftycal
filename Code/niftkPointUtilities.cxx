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
#include <niftkTypes.h>

namespace niftk {

//-----------------------------------------------------------------------------
PointSet CopyPoints(const PointSet& p)
{
  PointSet result;

  PointSet::const_iterator iter;
  for (iter = p.begin(); iter != p.end(); ++iter)
  {
    result.insert(IdPoint2D((*iter).first, (*iter).second));
  }

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
void ConvertPoints(const PointSet& input,
                   std::vector<cv::Point2f>& outputPoint,
                   std::vector<niftk::IdType>& outputId
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
                   const std::vector<niftk::IdType>& inputId,
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
                                                 const PointSet& inputB)
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
  for (size_t i = 0; i < a.size(); i++)
  {
    rms += ((a[i].x - b[i].x) * (a[i].x - b[i].x)
           +(a[i].y - b[i].y) * (a[i].y - b[i].y)
           );
  }
  if (a.size() > 0)
  {
    rms /= static_cast<double>(a.size());
  }
  rms = sqrt(rms);
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
  std::vector<niftk::IdType> ids;
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
  std::vector<niftk::IdType> ids;

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

  typedef std::pair<double, niftk::IdType> P;
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
  std::vector<niftk::IdType> leftUndistId;
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

} // end namespace
