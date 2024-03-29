/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRedBallDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
RedBallDetector::RedBallDetector()
{
}


//-----------------------------------------------------------------------------
RedBallDetector::~RedBallDetector()
{
}


//-----------------------------------------------------------------------------
void RedBallDetector::FillMask(const cv::Mat& imageToUse)
{
  if (imageToUse.channels() != 3)
  {
    niftkNiftyCalThrow() << "Should be 3 channel, BGR (standard OpenCV) image.";
  }

  cv::cvtColor(imageToUse, m_HSVImage, CV_BGR2HSV);

  // OpenCV hue values go from 0 to 180 and red wraps around,
  // so we look for 20 lowest and 20 highest.

  cv::inRange(m_HSVImage, cv::Scalar(0, 30, 30), cv::Scalar(20, 255, 255), m_Mask1);
  cv::inRange(m_HSVImage, cv::Scalar(160, 30, 30), cv::Scalar(180, 255, 255), m_Mask2);

  m_Mask = m_Mask1 | m_Mask2;
}

} // end namespace
