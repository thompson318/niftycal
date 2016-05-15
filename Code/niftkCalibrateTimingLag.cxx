/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCalibrateTimingLag.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-------------------------------------------------------------------
double CalibrateTimingOffset(
    std::list< std::pair<cv::Point2d, cv::Matx44d> >
    )
{
  niftkNiftyCalThrow() << "Not implemented yet";
  return 0;
}

} // end namespace