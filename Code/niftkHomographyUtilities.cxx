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
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
void WarpPointsByHomography(const PointSet& src,
                            const cv::Mat& h,
                            PointSet& target
                           )
{
  std::vector<cv::Point2f> srcP;
  std::vector<niftk::IdType> srcId;
  std::vector<cv::Point2f> targP;
  target.clear();

  niftk::ConvertPoints(src, srcP, srcId);
  cv::perspectiveTransform(srcP, targP, h);
  niftk::ConvertPoints(targP, srcId, target);
}


//-----------------------------------------------------------------------------
void FindHomography(const PointSet& src,
                    const PointSet& target,
                    const cv::Mat& homography
                   )
{
  niftkNiftyCalThrow() << "Not implemented yet";
}


//-----------------------------------------------------------------------------
void WarpImageByCorrespondingPoints(const cv::Mat& inputImage,
                                    const cv::Mat& cameraIntrinsics,
                                    const cv::Mat& distortionCoefficients,
                                    const PointSet& distortedPoints,
                                    const PointSet& targetPoints,
                                    const cv::Size2i outputImageSize,
                                    cv::Mat& outputHomography,
                                    cv::Mat& outputImage
                                   )
{
  std::vector<cv::Point2f> distortedSource;
  std::vector<cv::Point2f> undistortedSource;
  std::vector<cv::Point2f> target;

  niftk::ExtractCommonPoints(distortedPoints , targetPoints, distortedSource, target);

  if (distortedSource.size() == 0)
  {
    niftkNiftyCalThrow() << "No source points.";
  }
  if (target.size() == 0)
  {
    niftkNiftyCalThrow() << "No target points.";
  }
  if (distortedSource.size() != target.size())
  {
    niftkNiftyCalThrow() << "Different number of source and target points.";
  }

  if (cameraIntrinsics.rows == 3
      && cameraIntrinsics.cols == 3
      && distortionCoefficients.rows == 1
      && distortionCoefficients.cols == 4
      )
  {
    cv::undistortPoints(distortedSource, undistortedSource, cameraIntrinsics, distortionCoefficients);
    outputHomography = cv::findHomography(undistortedSource, target, 0);
  }
  else
  {
    outputHomography = cv::findHomography(distortedSource, target, 0);
  }
  niftk::WarpImageByHomography(inputImage, outputHomography, outputImageSize, outputImage);
}


//-----------------------------------------------------------------------------
void WarpImageByHomography(const cv::Mat& inputImage,
                           const cv::Mat& homography,
                           const cv::Size2i outputImageSize,
                           cv::Mat& outputImage
                          )
{
  cv::warpPerspective(inputImage, outputImage, homography, outputImageSize,cv::INTER_LANCZOS4);
}

} // end namespace
