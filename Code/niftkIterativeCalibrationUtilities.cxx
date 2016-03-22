/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeCalibrationUtilities_p.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
void ExtractTwoCopiesOfControlPoints(
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& list,
    std::list<PointSet>& a,
    std::list<PointSet>& b
    )
{
  a.clear();
  b.clear();

  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator iter;

  int counter = 0;
  for (iter = list.begin(); iter != list.end(); ++iter)
  {
    PointSet points = (*iter).first->GetPoints();
    if(points.empty())
    {
      niftkNiftyCalThrow() << "All input images should be valid calibration images containing extractable points, and " << counter << " isn't.";
    }

    a.push_back(points);
    b.push_back(points);
    counter++;
  }
}


//-----------------------------------------------------------------------------
PointSet ExtractDistortedControlPoints(
    const std::pair< cv::Mat, niftk::PointSet>& referenceData,
    const cv::Mat& intrinsic,
    const cv::Mat& distortion,
    const cv::Mat& originalImage,
    std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat>& outputDetectorAndImage,
    PointSet& outputPoints
    )
{
  cv::Mat undistortedImage;
  cv::Mat h;
  cv::Mat hInv;
  PointSet cp;
  PointSet cpi;
  PointSet cpid;

  cv::Size2i outputImageSize;
  outputImageSize.width = referenceData.first.cols;
  outputImageSize.height = referenceData.first.rows;

  // 1. Undistort and Unproject: Use the camera parameters to
  // undistort and unproject input images to a canonical pattern.
  cv::undistort(originalImage, undistortedImage, intrinsic, distortion, intrinsic);
  niftk::WarpImageByCorrespondingPoints(
        undistortedImage,
        intrinsic,                     // current estimate (updated each loop)
        distortion,                    // current estimate (updated each loop)
        outputPoints,                  // first time, its a copy of the original detected points,
                                       // after that, its updated, distorted points
        referenceData.second,          // specifies the target point locations
        outputImageSize,               // specifies the proposed size of warped image
        h,                             // output homography, written into
        outputDetectorAndImage.second  // output image, written into
        );

  // 2. Localize control points: Localize calibration pattern control
  // points in the canonical pattern.
  cp = outputDetectorAndImage.first->GetPoints();
  if(cp.empty())
  {
    niftkNiftyCalThrow() << "All warped images should still contain valid calibration images containing extractable points.";
  }

  // 2.5. Extract a window from the reference image, and do template matching.
/*
  int windowSize = 0;
  int scaleFactor = 10;
  cv::Mat referenceImageWindow;
  cv::Mat referenceImageResized;
  cv::Mat sourceImageWindow;
  cv::Mat sourceImageResized;
  cv::Mat result;

  niftk::PointSet::iterator iter;
  for (iter = cp.begin(); iter != cp.end(); ++iter)
  {
    cv::Point2d p = (*iter).second.point;

    if (windowSize == 0)
    {
      // Work out distance (pixels) to next reference point.
      // Assumption is that they are probably ordered in some way.
      niftk::PointSet::iterator iter2 = iter;
      ++iter2;
      cv::Point2d q = (*iter2).second.point;
      double distance = sqrt((p.x - q.x)*(p.x-q.x) + (p.y - q.y)*(p.y-q.y));
      windowSize = static_cast<int>(distance/3);
    }

    cv::Rect referenceRoi(static_cast<int>(p.x) - windowSize,  // x
                          static_cast<int>(p.y) - windowSize,  // y
                          2*windowSize + 1, // width
                          2*windowSize + 1 // height
                          );
    cv::Rect sourceRoi   (static_cast<int>(p.x) - windowSize -1,  // x
                          static_cast<int>(p.y) - windowSize -1,  // y
                          2*windowSize + 3, // width
                          2*windowSize + 3 // height
                          );

    referenceImageWindow = referenceData.first(referenceRoi);
    cv::resize(referenceImageWindow, referenceImageResized, cv::Size(), scaleFactor, scaleFactor);

    sourceImageWindow = outputDetectorAndImage.second(sourceRoi);
    cv::resize(sourceImageWindow, sourceImageResized, cv::Size(), scaleFactor, scaleFactor);

    result = cvCreateMat(
          sourceImageResized.cols - referenceImageResized.cols + 1,
          sourceImageResized.rows - referenceImageResized.rows + 1,
          CV_32FC1
          );

    cv::matchTemplate(sourceImageResized, referenceImageResized, result, CV_TM_CCORR_NORMED );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    double minVal; double maxVal; cv::Point2i minLoc; cv::Point2i maxLoc;
    cv::Point2i matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    matchLoc = maxLoc;

    // Convert match loc back to a real image location.
    (*iter).second.point.x = static_cast<double>(matchLoc.x)/static_cast<double>(scaleFactor) + p.x -1;
    (*iter).second.point.y = static_cast<double>(matchLoc.y)/static_cast<double>(scaleFactor) + p.y -1;
  }
*/

  // 3. Reproject: Project the control points using the estimated
  // camera parameters.
  hInv = h.inv(cv::DECOMP_SVD);
  niftk::WarpPointsByHomography(cp, hInv, cpi);
  niftk::DistortPoints(cpi, intrinsic, distortion, cpid);
  niftk::CopyPointsInto(cpid, outputPoints);

  PointSet trimmed = niftk::TrimPoints(cp, referenceData.second, 0.50);
  niftk::WarpPointsByHomography(trimmed, hInv, cpi);
  niftk::DistortPoints(cpi, intrinsic, distortion, cpid);
  niftk::CopyPointsInto(cpid, trimmed);

  return trimmed;
}

} // end namespace
