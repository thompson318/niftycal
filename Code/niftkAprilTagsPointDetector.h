/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkAprilTagsPointDetector_h
#define niftkAprilTagsPointDetector_h

#include "niftkWin32ExportHeader.h"
#include "niftkPointDetector.h"

namespace niftk
{

/**
* \class AprilTagsPointDetector
* \brief Detects AprilTags markers in a grey scale image.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT AprilTagsPointDetector : public PointDetector
{

public:

  AprilTagsPointDetector(bool includeCorners,
                         const std::string& name,
                         float sigma,
                         float segmentationSigma
                        );
  virtual ~AprilTagsPointDetector();

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:

  bool        m_IncludeCorners;
  std::string m_Name;
  float       m_Sigma;
  float       m_SegmentationSigma;
};

} // end namespace

#endif
