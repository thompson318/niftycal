/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkWhiteBallDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
WhiteBallDetector::WhiteBallDetector()
{
}


//-----------------------------------------------------------------------------
WhiteBallDetector::~WhiteBallDetector()
{
}


//-----------------------------------------------------------------------------
void WhiteBallDetector::FillMask(const cv::Mat& imageToUse)
{
  if (imageToUse.channels() != 3)
  {
    niftkNiftyCalThrow() << "Should be 3 channel, BGR (standard OpenCV) image.";
  }

  cv::cvtColor(imageToUse, m_GreyImage, CV_BGR2GRAY);
  cv::inRange(m_GreyImage, cv::Scalar(240), cv::Scalar(255), m_Mask);
}

} // end namespace
