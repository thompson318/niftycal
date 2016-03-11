/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMaxERE.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk {

//-----------------------------------------------------------------------------
double ComputeMaxEREFromCalibrations(
    const std::vector<cv::Point3f>& testPointsXYZ,
    const std::pair<cv::Mat, cv::Mat>& x,
    const std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    )
{
  double result = 0;
  return result;
}


//-----------------------------------------------------------------------------
void GetModelPosterior(
    const std::pair<cv::Mat, cv::Mat>& currentCalibration,
    std::pair<cv::Mat, cv::Mat>& x,
    std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    )
{

}


//-----------------------------------------------------------------------------
double ComputeMaxERE(
    const cv::Mat& intrinsic,
    const cv::Mat& distortion
    )
{
  double result = 0;
  return result;
}

} // end namespace
