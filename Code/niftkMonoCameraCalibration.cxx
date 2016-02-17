/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMonoCameraCalibration.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double MonoCameraCalibration(const Model3D& model,
                             const std::list<PointSet>& listOfPointSets,
                             cv::Matx33d& intrinsic,
                             cv::Matx16d& distortion,
                             std::list<cv::Matx44d>& extrinsic
                             )
{
  double rms = 1;
  return rms;
}

} // end namespace
