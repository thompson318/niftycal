/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTimingLagCalibration_h
#define niftkTimingLagCalibration_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <list>

namespace niftk
{

/**
* \brief Calibrate timing offset between a point extracted from video frames,
* and the corresponding tracker matrices.
*
* Note: Not implemented yet.
*/
NIFTYCAL_WINEXPORT double CalibrateTimingOffset(std::list< std::pair<cv::Point2d, cv::Matx44d> >& data);

} // end namespace

#endif
