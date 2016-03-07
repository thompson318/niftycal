/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkOpenCVCirclesPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <cv.h>
#include <highgui.h>

namespace niftk {

//-----------------------------------------------------------------------------
OpenCVCirclesPointDetector::OpenCVCirclesPointDetector(
    cv::Size2i patternSize)
: m_PatternSize(patternSize)
, m_Image(nullptr)
{
  if (m_PatternSize.width < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in width axes is too small.";
  }
  if (m_PatternSize.height < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in height axes is too small.";
  }
}


//-----------------------------------------------------------------------------
OpenCVCirclesPointDetector::~OpenCVCirclesPointDetector()
{
}


//-----------------------------------------------------------------------------
void OpenCVCirclesPointDetector::SetImage(cv::Mat* image)
{
  if (image == nullptr)
  {
    niftkNiftyCalThrow() << "Null image provided.";
  }
  m_Image = image;
}


//-----------------------------------------------------------------------------
PointSet OpenCVCirclesPointDetector::GetPoints()
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image is Null.";
  }

  PointSet result;
  std::vector<cv::Point2f> circles;
  unsigned int numberOfCircles = m_PatternSize.width * m_PatternSize.height;

  /** http://answers.opencv.org/question/3471/findcirclesgrid-not-working/ */
  cv::SimpleBlobDetector::Params params;
  params.maxArea = 10e4;
  cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);

  bool found = cv::findCirclesGrid(
    *m_Image, m_PatternSize, circles,
    cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    , blobDetector
    );

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
//      std::cerr << tmp.id << " " << tmp.point.x << " " << tmp.point.y << std::endl;
    }
  }

//  cv::drawChessboardCorners(*m_Image, m_PatternSize, circles, found);
//  cv::imwrite("/tmp/matt2.png", *m_Image);

  return result;
}

} // end namespace
