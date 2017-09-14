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

#include <highgui.h>

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

  cv::medianBlur(m_Mask, m_MedianSmoothed, 13);
  cv::GaussianBlur(m_MedianSmoothed, m_GaussianSmoothed, cv::Size(5,5), 3, 3);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(m_GaussianSmoothed, circles, CV_HOUGH_GRADIENT, 1, 100, 200, 20, 0, 0);

  niftk::PointSet result;

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
