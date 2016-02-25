/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkHomographyUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
void WarpImageByCorrespondingPoints(const cv::Mat& inputImage,
                                    const PointSet& sourcePoints,
                                    const PointSet& targetPoints,
                                    const cv::Size2i outputImageSize,
                                    cv::Mat& outputImage
                                   )
{
  std::vector<cv::Point2f> source;
  std::vector<cv::Point2f> target;

  niftk::PointSet::const_iterator sourceIter;
  niftk::PointSet::const_iterator targetIter;

  for(sourceIter = sourcePoints.begin(); sourceIter != sourcePoints.end(); ++sourceIter)
  {
    targetIter = targetPoints.find((*sourceIter).first);
    if (targetIter != targetPoints.end())
    {
      cv::Point2f s;
      s.x = (*sourceIter).second.point.x;
      s.y = (*sourceIter).second.point.y;
      source.push_back(s);

      cv::Point2f t;
      t.x = (*targetIter).second.point.x;
      t.y = (*targetIter).second.point.y;
      target.push_back(t);
    }
  }
  if (source.size() == 0)
  {
    niftkNiftyCalThrow() << "No source points.";
  }
  if (target.size() == 0)
  {
    niftkNiftyCalThrow() << "No target points.";
  }
  if (source.size() != target.size())
  {
    niftkNiftyCalThrow() << "Different number of source and target points.";
  }

  cv::Mat homography = cv::findHomography(  source, target, 0 );
  niftk::WarpImageByHomography(inputImage, homography, outputImageSize, outputImage);
}


//-----------------------------------------------------------------------------
void WarpImageByHomography(const cv::Mat& inputImage,
                           const cv::Mat& homography,
                           const cv::Size2i outputImageSize,
                           cv::Mat& outputImage
                          )
{
  cv::warpPerspective(inputImage,outputImage,homography,outputImageSize,cv::INTER_LINEAR);
}

} // end namespace
