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
#include <niftkIPoint2DDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class OpenCVChessboardPointDetector
* \brief Detects complete chessboards in a grey scale image, using OpenCV.
*
* Note, this detector does no image conversion.
*/
class NIFTYCAL_WINEXPORT OpenCVChessboardPointDetector : public IPoint2DDetector
{

public:

  OpenCVChessboardPointDetector(cv::Mat* greyImage, cv::Size2i numberOfCorners);
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
