/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkAprilTagsPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk {

//-----------------------------------------------------------------------------
AprilTagsPointDetector::AprilTagsPointDetector(
    cv::Mat* image)
: m_Image(image)
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image is NULL.";
  }
}


//-----------------------------------------------------------------------------
AprilTagsPointDetector::~AprilTagsPointDetector()
{
  // Do NOT destroy m_Image, we don't own it.
}


//-----------------------------------------------------------------------------
PointSet AprilTagsPointDetector::GetPoints()
{
  PointSet result;

  return result;
}

} // end namespace
