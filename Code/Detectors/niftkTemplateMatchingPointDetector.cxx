/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTemplateMatchingPointDetector.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkHomographyUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkTemplateMatching.h>
#include <cv.h>
#include <highgui.h>

namespace niftk {

//-----------------------------------------------------------------------------
TemplateMatchingPointDetector::TemplateMatchingPointDetector(cv::Size2i patternSize,
                                                             cv::Size2i offsetForTemplate,
                                                             int flags
                                                            )
: m_OffsetForTemplate(offsetForTemplate)
, m_Flags(flags)
, m_MaxAreaInPixels(10000)
, m_UseContours(true)
, m_UseInternalResampling(true)
, m_UseTemplateMatching(true)
, m_ReferenceImage(nullptr)
, m_TemplateImage(nullptr)
{
  // This is intentionally swapped round.
  // Default OpenCV asymetric circle detection appears
  // to not work the other way round.
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
  if (m_OffsetForTemplate.width < 1)
  {
    niftkNiftyCalThrow() << "Offset width must be >= 1";
  }
  if (m_OffsetForTemplate.height < 1)
  {
    niftkNiftyCalThrow() << "Offset height must be >= 1";
  }
}


//-----------------------------------------------------------------------------
TemplateMatchingPointDetector::~TemplateMatchingPointDetector()
{
}


//-----------------------------------------------------------------------------
int TemplateMatchingPointDetector::GetFlags()
{
  return m_Flags;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetMaxAreaInPixels(unsigned long int& pixels)
{
  m_MaxAreaInPixels = pixels;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetUseContours(bool useContours)
{
  m_UseContours = useContours;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetUseTemplateMatching(bool useTemplateMatching)
{
  m_UseTemplateMatching = useTemplateMatching;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetUseInternalResampling(bool useResampling)
{
  m_UseInternalResampling = useResampling;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetReferencePoints(const niftk::PointSet& points)
{
  m_ReferencePoints = points;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetReferenceImage(cv::Mat* image)
{
  m_ReferenceImage = image;
}


//-----------------------------------------------------------------------------
void TemplateMatchingPointDetector::SetTemplateImage(cv::Mat* image)
{
  m_TemplateImage = image;
}


//-----------------------------------------------------------------------------
PointSet TemplateMatchingPointDetector::GetPointsUsingTemplateMatching(const cv::Mat& image,
                                                                       const niftk::PointSet& startingGuess
                                                                      )
{
  niftk::PointSet result;

  if (m_UseInternalResampling)
  {
    cv::Mat homography;
    niftk::FindHomography(startingGuess, m_ReferencePoints, homography);

    cv::Mat warpedImage;
    cv::warpPerspective(image, warpedImage, homography, m_ReferenceImage->size(), cv::INTER_LINEAR);

    niftk::PointSet warpedPoints;
    niftk::WarpPointsByHomography(startingGuess, homography, warpedPoints);

    niftk::PointSet matchedPoints = niftk::DoTemplateMatchingForAllPoints(warpedImage,
                                                                          *m_TemplateImage,
                                                                          m_OffsetForTemplate,
                                                                          warpedPoints);

    niftk::WarpPointsByHomography(matchedPoints, homography.inv(), result);
  }
  else
  {
    result = niftk::DoTemplateMatchingForAllPoints(image,
                                                   *m_TemplateImage,
                                                   m_OffsetForTemplate,
                                                   startingGuess
                                                  );
  }
  return result;
}


//-----------------------------------------------------------------------------
PointSet TemplateMatchingPointDetector::InternalGetPoints(const cv::Mat& imageToUse)
{
  if (!m_UseContours && !m_UseTemplateMatching)
  {
    niftkNiftyCalThrow() << "You must chose contour detection or template matching.";
  }

  niftk::PointSet result;

  if (m_UseContours || m_InitialGuess.size() == 0)
  {
    result = this->GetPointsUsingContours(imageToUse);
  }

  if (m_UseTemplateMatching)
  {
    if (!m_InitialGuess.empty())
    {
      if (m_InitialGuess.size() != m_PatternSize.width * m_PatternSize.height)
      {
        niftkNiftyCalThrow() << "Initial guess contains the wrong number of points.";
      }
      result = this->GetPointsUsingTemplateMatching(imageToUse, m_InitialGuess);
    }
    else
    {
      if (!result.empty())
      {
        result = this->GetPointsUsingTemplateMatching(imageToUse, result);
      }
    }
  }
  return result;
}

} // end namespace
