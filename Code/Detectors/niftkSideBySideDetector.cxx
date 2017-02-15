/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkSideBySideDetector.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
SideBySideDetector::SideBySideDetector(std::unique_ptr<niftk::PointDetector>& left,
                                       std::unique_ptr<niftk::PointDetector>& right
                                      )
: m_SequentialNumbering(true)
{
  m_LeftDetector = std::move(left);
  m_RightDetector = std::move(right);
}


//-----------------------------------------------------------------------------
SideBySideDetector::~SideBySideDetector()
{
  // smart pointers destroy
}


//-----------------------------------------------------------------------------
PointSet SideBySideDetector::InternalGetPoints(const cv::Mat& imageToUse)
{
  niftk::PointSet result;

  // These create a new header rather than copying data.
  cv::Mat leftImage = imageToUse(cv::Rect(0,
                                          0,
                                          imageToUse.cols / 2,
                                          imageToUse.rows)
                                );

  cv::Mat rightImage = imageToUse(cv::Rect(imageToUse.cols / 2,
                                           0,
                                           imageToUse.cols - (imageToUse.cols/2),
                                           imageToUse.rows)
                                 );

  // Assume input image is already scaled.
  cv::Point2d scaleFactor;
  scaleFactor.x = 1;
  scaleFactor.y = 1;

  m_LeftDetector->SetImage(&leftImage);
  m_LeftDetector->SetImageScaleFactor(scaleFactor);
  m_LeftDetector->SetCaching(false);
  result = m_LeftDetector->GetPoints();

  m_RightDetector->SetImage(&rightImage);
  m_RightDetector->SetImageScaleFactor(scaleFactor);
  m_RightDetector->SetCaching(false);
  niftk::PointSet rightPoints = m_RightDetector->GetPoints();

  unsigned int numberOfLeftHandPoints = result.size();

  // Combine points to produce output.

  niftk::PointSet::const_iterator iter;
  for (iter = rightPoints.begin(); iter != rightPoints.end(); ++iter)
  {
    niftk::NiftyCalIdType id = (*iter).first;
    niftk::Point2D p = (*iter).second;
    if (m_SequentialNumbering)
    {
      p.id = id + numberOfLeftHandPoints;
    }
    else
    {
      p.id = id;
    }
    result.insert(niftk::IdPoint2D(p.id, p));
  }

  return result;
}

} // end namespace
