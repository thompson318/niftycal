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
#include <niftkIPointDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class OpenCVChessboardPointDetector
* \brief Detects complete chessboards in an image, using OpenCV.
*/
class NIFTYCAL_WINEXPORT OpenCVChessboardPointDetector : public IPointDetector
{

public:

  OpenCVChessboardPointDetector(cv::Mat* image, cv::Size2i numberOfCorners);
  virtual ~OpenCVChessboardPointDetector();

  /**
  * \see IPointDetector::GetPoints()
  */
  virtual PointSet GetPoints();

private:

  cv::Size2i m_NumberOfCorners;
  cv::Mat*   m_Image; // non-owning
};

} // end namespace

#endif
