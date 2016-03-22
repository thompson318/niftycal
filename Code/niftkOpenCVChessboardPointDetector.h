/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkOpenCVChessboardPointDetector_h
#define niftkOpenCVChessboardPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <niftkOpenCVPointDetector.h>

namespace niftk
{

/**
* \class OpenCVChessboardPointDetector
* \brief Detects complete OpenCV chessboards in a grey scale image using cv::findChessboardCorners.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVChessboardPointDetector : public OpenCVPointDetector
{

public:

  OpenCVChessboardPointDetector(cv::Size2i numberOfCorners);
  virtual ~OpenCVChessboardPointDetector();

protected:
  /**
  * \see niftk::OpenCVPointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:
  cv::Size2i m_NumberOfCorners;
};

} // end namespace

#endif
