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
#include <niftkIPoint2DDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class AprilTagsPointDetector
* \brief Detects AprilTags markers in a grey scale image.
*
* Note, this detector does no image conversion.
*/
class NIFTYCAL_WINEXPORT AprilTagsPointDetector : public IPoint2DDetector
{

public:

  AprilTagsPointDetector(cv::Mat* greyScaleImage,
                         bool includeCorners,
                         const std::string& name,
                         float sigma,
                         float segmentationSigma
                         );
  virtual ~AprilTagsPointDetector();

  /**
  * \see IPointDetector::GetPoints()
  */
  virtual PointSet GetPoints();

private:

  cv::Mat*    m_Image; // non-owning
  bool        m_IncludeCorners;
  std::string m_Name;
  float       m_Sigma;
  float       m_SegmentationSigma;
};

} // end namespace

#endif
