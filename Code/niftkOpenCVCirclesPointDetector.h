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
#include <niftkIPoint2DDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class OpenCVCirclesPointDetector
* \brief Detects asymmetric circles pattern in grey scale images using cv::findCirclesGrid.
*
* Note, this detector does no image conversion.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVCirclesPointDetector : public IPoint2DDetector
{

public:

  OpenCVCirclesPointDetector(cv::Size2i patternSize);
  virtual ~OpenCVCirclesPointDetector();

  /**
  * \see IPointDetector::GetPoints()
  */
  virtual PointSet GetPoints();

  void SetImage(cv::Mat* image);

private:

  cv::Size2i m_PatternSize;
  cv::Mat*   m_Image;
};

} // end namespace

#endif
