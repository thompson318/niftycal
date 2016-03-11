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
* \brief Detects complete OpenCV chessboards in a grey scale image, cv::findChessboardCorners.
*
* Note, this detector does no image conversion.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVChessboardPointDetector : public IPoint2DDetector
{

public:

  OpenCVChessboardPointDetector(cv::Size2i numberOfCorners);
  virtual ~OpenCVChessboardPointDetector();

  /**
  * \see IPointDetector::GetPoints()
  */
  virtual PointSet GetPoints();

  void SetImage(cv::Mat* image);

private:

  cv::Size2i m_NumberOfCorners;
  cv::Mat*   m_Image;
};

} // end namespace

#endif
