/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkOpenCVPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkPointUtilities.h"

namespace niftk {

//-----------------------------------------------------------------------------
OpenCVPointDetector::OpenCVPointDetector()
: m_Image(nullptr)
{
  m_ScaleFactors.x = 1;
  m_ScaleFactors.y = 1;
}


//-----------------------------------------------------------------------------
OpenCVPointDetector::~OpenCVPointDetector()
{
}


//-----------------------------------------------------------------------------
void OpenCVPointDetector::SetImage(cv::Mat* image)
{
  if (image == nullptr)
  {
    niftkNiftyCalThrow() << "Null image provided.";
  }
  m_Image = image;
}


//-----------------------------------------------------------------------------
void OpenCVPointDetector::SetImageScaleFactor(const cv::Point2d& scaleFactor)
{
  if (scaleFactor.x <= 0)
  {
    niftkNiftyCalThrow() << "X scale factor should be > 0.";
  }
  if (scaleFactor.y <= 0)
  {
    niftkNiftyCalThrow() << "Y scale factor should be > 0.";
  }
  m_ScaleFactors = scaleFactor;
}


//-----------------------------------------------------------------------------
PointSet OpenCVPointDetector::GetPoints()
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image is Null.";
  }

  PointSet result;

  if (m_ScaleFactors.x == 1 && m_ScaleFactors.y == 1)
  {
    result = InternalGetPoints(*m_Image);
  }
  else
  {
    cv::resize(*m_Image, m_RescaledImage, cv::Size(0, 0), m_ScaleFactors.x, m_ScaleFactors.y, cv::INTER_LINEAR);
    PointSet points = InternalGetPoints(m_RescaledImage);

    cv::Point2d scaleDownFactors;
    scaleDownFactors.x = 1.0/m_ScaleFactors.x;
    scaleDownFactors.y = 1.0/m_ScaleFactors.y;
    result = niftk::RescalePoints(points, scaleDownFactors);
  }
  return result;
}

} // end namespace
