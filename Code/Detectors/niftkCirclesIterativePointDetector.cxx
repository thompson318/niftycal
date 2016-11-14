/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCirclesIterativePointDetector.h"
#include <cv.h>

namespace niftk {

//-----------------------------------------------------------------------------
CirclesIterativePointDetector::CirclesIterativePointDetector(cv::Size2i patternSize,
                                                             cv::Size2i offsetForTemplate
                                                            )
: TemplateMatchingPointDetector(patternSize, offsetForTemplate)
{
}


//-----------------------------------------------------------------------------
CirclesIterativePointDetector::~CirclesIterativePointDetector()
{
}


//-----------------------------------------------------------------------------
PointSet CirclesIterativePointDetector::GetPointsUsingContours(const cv::Mat& image)
{
  PointSet result;
  unsigned int numberOfDots = m_PatternSize.width * m_PatternSize.height;

  cv::SimpleBlobDetector::Params params;
  params.maxArea = m_MaxAreaInPixels;
  cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);

  std::vector<cv::Point2f> centres;
  bool found = cv::findCirclesGrid(
    image, m_PatternSize, centres,
    cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    , blobDetector
    );

  if (!found)
  {
    return result;
  }

  assert(centres.size() == numberOfDots);

  for ( unsigned int k = 0; k < numberOfDots; ++k)
  {
    Point2D tmp;
    tmp.point.x = centres[k].x;
    tmp.point.y = centres[k].y;
    tmp.id = k;
    result.insert(IdPoint2D(tmp.id, tmp));
  }
  return result;
}

} // end namespace
