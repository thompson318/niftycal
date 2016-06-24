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

  /**
  * \brief Constructor.
  * \param includeCorners if true will output the 4 corners as well as
  * the centre point. Preliminary testing suggests that the central point
  * is most accurate, as there is an averaging effect of computing the
  * centre from the 4 corners. So, if you want really accurate points,
  * just use the centre one. If you want lots of additional points,
  * use all the corners aswell. The corners are ordered, and given a
  * point ID, are output as 1000+ID, 2000+ID, 3000+ID and 4000+ID.
  */
  AprilTagsPointDetector(bool includeCorners,
                         const std::string& tagFamilyName,
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
