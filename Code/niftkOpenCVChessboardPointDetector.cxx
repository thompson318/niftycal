/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkOpenCVChessboardPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk {

//-----------------------------------------------------------------------------
OpenCVChessboardPointDetector::OpenCVChessboardPointDetector(
    cv::Mat* image,
    cv::Size2i numberOfCorners)
: m_NumberOfCorners(numberOfCorners)
, m_Image(image)
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image is NULL";
  }
}


//-----------------------------------------------------------------------------
OpenCVChessboardPointDetector::~OpenCVChessboardPointDetector()
{
  // Do NOT destroy m_Image, we don't own it.
}


//-----------------------------------------------------------------------------
PointSet OpenCVChessboardPointDetector::GetPoints()
{
  PointSet result;
  std::vector<cv::Point2f> corners;

  bool found = cv::findChessboardCorners(
        *m_Image, m_NumberOfCorners, corners,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

  if ( corners.size() == 0 )
  {
    return result;
  }

  unsigned int numberOfCorners = m_NumberOfCorners.width * m_NumberOfCorners.height;

  cv::Mat greyImage;
  cv::cvtColor(*m_Image, greyImage, CV_BGR2GRAY);

  cv::cornerSubPix(greyImage, corners, cv::Size(11,11), cv::Size(-1,-1),
                   cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

  if (found  && corners.size() == numberOfCorners)
  {

    for ( unsigned int k = 0; k < numberOfCorners; ++k)
    {
      Point2D tmp;
      tmp.point.x = corners[k].x;
      tmp.point.y = corners[k].y;
      tmp.id = k;
      result.insert(IdPoint(tmp.id, tmp));
    }
  }

  return result;
}

} // end namespace
