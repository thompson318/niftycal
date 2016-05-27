/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkOpenCVRingsPointDetector_h
#define niftkOpenCVRingsPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <niftkOpenCVPointDetector.h>

namespace niftk
{

/**
* \class OpenCVRingsPointDetector
* \brief Detects rings pattern, as seen in Dutta 2009
* http://dx.doi.org/10.1109/ICCVW.2009.5457474
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVRingsPointDetector : public OpenCVPointDetector
{

public:

  OpenCVRingsPointDetector(cv::Size2i patternSize);
  virtual ~OpenCVRingsPointDetector();

protected:

  /**
  * \see niftk::OpenCVPointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:

  void ExtractBlobs(const cv::Mat& image,
                    cv::Mat& bigBlobs,
                    cv::Mat& littleBlobs
                   );

  PointSet GetPointsUsingContours(const cv::Mat& image);

  cv::Size2i m_PatternSize;
};

} // end namespace

#endif
