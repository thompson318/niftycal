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
#include <niftkIPointDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class AprilTagsPointDetector
* \brief Detects AprilTags markers in an image.
*/
class NIFTYCAL_WINEXPORT AprilTagsPointDetector : public IPointDetector
{

public:

  AprilTagsPointDetector(cv::Mat* image);
  virtual ~AprilTagsPointDetector();

  /**
  * \see IPointDetector::GetPoints()
  */
  virtual PointSet GetPoints();

private:

  cv::Mat*   m_Image; // non-owning
};

} // end namespace

#endif
