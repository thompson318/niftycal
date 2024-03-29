/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCirclesPointDetector.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <cv.h>

namespace niftk {

//-----------------------------------------------------------------------------
CirclesPointDetector::CirclesPointDetector(cv::Size2i patternSize, int flags)
: m_Flags(flags)
{
  // This is intentionally swapped round.
  // Asymetric circle detection appears to not work the other way round.
  m_PatternSize.width = patternSize.height;
  m_PatternSize.height = patternSize.width;

  if (m_PatternSize.width < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in width axes is < 2.";
  }
  if (m_PatternSize.height < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in height axes is < 2.";
  }  
}


//-----------------------------------------------------------------------------
CirclesPointDetector::~CirclesPointDetector()
{
}


//-----------------------------------------------------------------------------
PointSet CirclesPointDetector::InternalGetPoints(const cv::Mat& imageToUse)
{
  PointSet result;
  std::vector<cv::Point2f> circles;
  unsigned int numberOfCircles = m_PatternSize.width * m_PatternSize.height;

  /** http://answers..org/question/3471/findcirclesgrid-not-working/ */
  cv::SimpleBlobDetector::Params params;
  params.maxArea = 10e4;
  cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);

  bool found = cv::findCirclesGrid(imageToUse, m_PatternSize, circles, m_Flags, blobDetector);
  if ( circles.size() == 0 )
  {
    return result;
  }

  if (found  && circles.size() == numberOfCircles)
  {
    for ( unsigned int k = 0; k < numberOfCircles; ++k)
    {
      Point2D tmp;
      tmp.point.x = circles[k].x;
      tmp.point.y = circles[k].y;
      tmp.id = k;
      result.insert(IdPoint2D(tmp.id, tmp));
    }
  }
  return result;
}

} // end namespace
