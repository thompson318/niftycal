/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkCirclesPointDetector_h
#define niftkCirclesPointDetector_h

#include <niftkWin32ExportHeader.h>
#include <niftkPointDetector.h>

namespace niftk
{

/**
* \class CirclesPointDetector
* \brief Detects circles pattern in grey scale images using OpenCV cv::findCirclesGrid.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT CirclesPointDetector : public PointDetector
{

public:

  CirclesPointDetector(cv::Size2i patternSize,
                       int flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

  virtual ~CirclesPointDetector();

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:

  cv::Size2i m_PatternSize;
  int        m_Flags;
};

} // end namespace

#endif
