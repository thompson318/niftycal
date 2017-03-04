/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTemplateCirclesPointDetector.h"
#include <cv.h>

namespace niftk {

//-----------------------------------------------------------------------------
TemplateCirclesPointDetector::TemplateCirclesPointDetector(cv::Size2i patternSize,
                                                           cv::Size2i offsetForTemplate,
                                                           int flags
                                                          )
: TemplateMatchingPointDetector(patternSize, offsetForTemplate, flags)
{
  // This is intentionally swapped round.
  // Default OpenCV asymetric circle detection appears
  // to not work the other way round.
  m_PatternSize.width = patternSize.height;
  m_PatternSize.height = patternSize.width;
}


//-----------------------------------------------------------------------------
TemplateCirclesPointDetector::~TemplateCirclesPointDetector()
{
}


//-----------------------------------------------------------------------------
PointSet TemplateCirclesPointDetector::GetPointsUsingContours(const cv::Mat& image)
{
  PointSet result;
  unsigned int numberOfDots = m_PatternSize.width * m_PatternSize.height;

  cv::SimpleBlobDetector::Params params;
  params.maxArea = m_MaxAreaInPixels;
  cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);

  std::vector<cv::Point2f> centres;
  bool found = cv::findCirclesGrid(
    image, m_PatternSize, centres,
    this->GetFlags(), blobDetector
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
