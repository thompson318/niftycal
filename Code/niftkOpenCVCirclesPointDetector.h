/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkOpenCVCirclesPointDetector_h
#define niftkOpenCVCirclesPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <niftkOpenCVPointDetector.h>

namespace niftk
{

/**
* \class OpenCVCirclesPointDetector
* \brief Detects asymmetric circles pattern in grey scale images using cv::findCirclesGrid.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVCirclesPointDetector : public OpenCVPointDetector
{

public:

  OpenCVCirclesPointDetector(cv::Size2i patternSize);
  virtual ~OpenCVCirclesPointDetector();

protected:
  /**
  * \see niftk::OpenCVPointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:
  cv::Size2i m_PatternSize;
};

} // end namespace

#endif
