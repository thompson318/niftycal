/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkSideBySideDetector.h"

namespace niftk
{

//-----------------------------------------------------------------------------
SideBySideDetector::SideBySideDetector(std::unique_ptr<niftk::PointDetector>& left,
                                       std::unique_ptr<niftk::PointDetector>& right
                                      )
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
  return result;
}

} // end namespace
