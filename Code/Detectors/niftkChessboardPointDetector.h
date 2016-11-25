/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkChessboardPointDetector_h
#define niftkChessboardPointDetector_h

#include <niftkWin32ExportHeader.h>
#include <niftkPointDetector.h>

namespace niftk
{

/**
* \class ChessboardPointDetector
* \brief Detects complete chessboards in a grey scale image using OpenCV cv::findChessboardCorners.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT ChessboardPointDetector : public PointDetector
{

public:

  ChessboardPointDetector(cv::Size2i numberOfCorners);
  virtual ~ChessboardPointDetector();

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:

  cv::Size2i m_NumberOfCorners;
};

} // end namespace

#endif
