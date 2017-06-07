/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkBallDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
BallDetector::BallDetector()
{
}


//-----------------------------------------------------------------------------
BallDetector::~BallDetector()
{
}


//-----------------------------------------------------------------------------
PointSet BallDetector::InternalGetPoints(const cv::Mat& imageToUse)
{
  this->FillMask(imageToUse);

  niftk::PointSet result;

  cv::GaussianBlur(m_Mask, m_GaussianSmoothed, cv::Size(5,5), 1, 1);
  cv::medianBlur(m_GaussianSmoothed, m_MedianSmoothed, 5);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(m_MedianSmoothed, circles, CV_HOUGH_GRADIENT, 1, m_MedianSmoothed.rows/8, 200, 100);

  for (int i = 0; i < circles.size(); i++)
  {
    cv::Point2d p;
    p.x = circles[i][0];
    p.y = circles[i][1];

    Point2D point;
    point.id = i;
    point.point = p;

    result.insert(IdPoint2D(i, point));
  }

  return result;
}

} // end namespace
