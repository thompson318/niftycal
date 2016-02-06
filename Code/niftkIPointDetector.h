/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIPointDetector_h
#define niftkIPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>

namespace niftk
{

/**
* \class IPointDetector
* \brief Interface for anything that detects points in an image.
*/
class NIFTYCAL_WINEXPORT IPointDetector
{

public:

  IPointDetector();
  virtual ~IPointDetector();

  /**
  * \brief Retrieves points, each one identified by a single id.
  * \return vector of id,point pairs.
  */
  virtual std::vector< std::pair<unsigned int, cv::Vec2d> > GetPoints() = 0;

};

} // end namespace

#endif
