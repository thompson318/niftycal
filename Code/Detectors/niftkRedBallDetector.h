/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRedBallDetector_h
#define niftkRedBallDetector_h

#include <niftkWin32ExportHeader.h>
#include "niftkBallDetector.h"

namespace niftk
{
/**
* \class RedBallDetector
* \brief Thresholds red things, so base class can fit circles.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT RedBallDetector : public BallDetector {

public:

  RedBallDetector();
  virtual ~RedBallDetector();

protected:

  /**
  * \see niftk::BallDetector::FillMask()
  */
  virtual void FillMask(const cv::Mat& imageToUse);

private:

  cv::Mat1b m_Mask1;
  cv::Mat1b m_Mask2;
  cv::Mat   m_HSVImage;

};

} // end namespace

#endif
