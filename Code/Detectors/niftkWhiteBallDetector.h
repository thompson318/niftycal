/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkWhiteBallDetector_h
#define niftkWhiteBallDetector_h

#include <niftkWin32ExportHeader.h>
#include "niftkBallDetector.h"

namespace niftk
{
/**
* \class BallDetector
* \brief Thresholds red things, so base class can fit circles.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT WhiteBallDetector : public BallDetector {

public:

  WhiteBallDetector();
  virtual ~WhiteBallDetector();

protected:

  /**
  * \see niftk::BallDetector::FillMask()
  */
  virtual void FillMask(const cv::Mat& imageToUse);

private:

  cv::Mat   m_HSVImage;

};

} // end namespace

#endif
