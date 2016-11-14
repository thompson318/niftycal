/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkPointDetector.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>

namespace niftk {

//-----------------------------------------------------------------------------
PointDetector::PointDetector()
: m_Image(nullptr)
, m_Caching(false)
, m_NeedsUpdating(true)
{
  m_ScaleFactors.x = 1;
  m_ScaleFactors.y = 1;
}


//-----------------------------------------------------------------------------
PointDetector::~PointDetector()
{
}


//-----------------------------------------------------------------------------
void PointDetector::SetCaching(const bool& isCaching)
{
  m_Caching = isCaching;
  m_NeedsUpdating = true;
}


//-----------------------------------------------------------------------------
void PointDetector::SetImage(cv::Mat* image)
{
  if (image == nullptr)
  {
    niftkNiftyCalThrow() << "Null image provided.";
  }
  m_Image = image;
  m_NeedsUpdating = true;
}


//-----------------------------------------------------------------------------
void PointDetector::SetImageScaleFactor(const cv::Point2d& scaleFactor)
{
  if (scaleFactor.x <= 0)
  {
    niftkNiftyCalThrow() << "X scale factor <= 0.";
  }
  if (scaleFactor.y <= 0)
  {
    niftkNiftyCalThrow() << "Y scale factor <= 0.";
  }
  m_ScaleFactors = scaleFactor;
  m_NeedsUpdating = true;
}


//-----------------------------------------------------------------------------
void PointDetector::SetInitialGuess(const PointSet& guess)
{
  m_InitialGuess = guess;
  m_NeedsUpdating = true;
}


//-----------------------------------------------------------------------------
PointSet PointDetector::GetPoints()
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image is Null.";
  }

  if (m_Caching && !m_NeedsUpdating)
  {
    return m_CachedResult; // even if its empty, i.e. previous attempt returned zero points.
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
  m_CachedResult = result;
  m_NeedsUpdating = false;
  return result;
}

} // end namespace
