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

  double rms = 0;
  return rms;
}


//-----------------------------------------------------------------------------
void UndistortPoints(const PointSet& distortedPoints,
                     const cv::Mat& cameraIntrinsics,
                     const cv::Mat& distortionCoefficients,
                     PointSet& undistortedPoints
                    )
{

  std::vector<cv::Point2f> distorted;
  std::vector<cv::Point2f> undistorted;
  std::vector<niftk::IdType> ids;

  niftk::ConvertPoints(distortedPoints, distorted, ids);
  cv::undistort(distorted, undistorted, cameraIntrinsics, distortionCoefficients);
  niftk::ConvertPoints(undistorted, ids, undistortedPoints);
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
    double radial = (1 + distortionCoefficients.at<double>(0,0) * r2 + distortionCoefficients.at<double>(0,1) * r2 * r2 );

    cv::Point2d dist;
    dist.x = relative.x * radial;
    dist.y = relative.y * radial;

    dist.x = dist.x + (2 * distortionCoefficients.at<double>(0,3) * relative.x * relative.y + distortionCoefficients.at<double>(0,4) * (r2 + 2 * relative.x * relative.x));
    dist.y = dist.y + (distortionCoefficients.at<double>(0,3) * (r2 + 2 * relative.y * relative.y) + 2 * distortionCoefficients.at<double>(0,4) * relative.x * relative.y);

    dist.x = dist.x * cameraIntrinsics.at<double>(0,0) + cameraIntrinsics.at<double>(0,2);
    dist.y = dist.y * cameraIntrinsics.at<double>(1,1) + cameraIntrinsics.at<double>(1,2);

    distorted.push_back(dist);
  }
  niftk::ConvertPoints(distorted, ids, distortedPoints);
}

} // end namespace
