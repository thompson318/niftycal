/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkBallDetector_h
#define niftkBallDetector_h

#include <niftkWin32ExportHeader.h>
#include "niftkPointDetector.h"

namespace niftk
{
/**
* \class BallDetector
* \brief Assuming derived classes threshold blobs, fits hough circle, returns centre point.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT BallDetector : public PointDetector {

public:

  BallDetector();
  virtual ~BallDetector();

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

  /**
  * \brief Derived classes do initial thresholding of imageToUse to populate m_Mask.
  */
  virtual void FillMask(const cv::Mat& imageToUse) = 0;

  cv::Mat1b m_Mask;

private:

  cv::Mat m_GaussianSmoothed;
  cv::Mat m_MedianSmoothed;
};

} // end namespace

#endif
