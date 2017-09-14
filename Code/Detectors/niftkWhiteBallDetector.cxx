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
#include <highgui.h>

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

  cv::cvtColor(imageToUse, m_HSVImage, CV_BGR2HSV);
  cv::inRange(m_HSVImage, cv::Scalar(0, 0, 245), cv::Scalar(2, 2, 255), m_Mask);

}

} // end namespace
