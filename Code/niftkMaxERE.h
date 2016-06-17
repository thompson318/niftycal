/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkMaxERE_h
#define niftkMaxERE_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"

/**
* \file niftkMaxERE.h
* \brief Functions to calculate MaxERE.
*
* See Algorithm 1, <a href="http://dx.doi.org/10.1109/IROS.2013.6696595">Richardson et al. IROS 2013</a>.
*
*/
namespace niftk
{

/**
* \brief Calculates MaxERE from a set of sample calibrations.
* \param x mean calibration, specified as (intrinsic(3,3), distortion(1,4))
* \param testPointsXYZ grid of points eg. 5x5 test points
* \param calSamples vector of sample calibrations, specified as (intrinsic(3,3), distortion(1,4))
*/
NIFTYCAL_WINEXPORT double ComputeMaxEREFromCalibrations(
    const std::vector<cv::Point3f>& testPointsXYZ,
    const std::pair<cv::Mat, cv::Mat>& x,
    const std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    );

/**
* \brief Produces a mean calibration and samples from posterior distribution.
* \param currentCalibration specified as (intrinsic(3,3), distortion(1,4))
* \param x mean calibration
* \param calSamples samples from the posterior distribution
*/
NIFTYCAL_WINEXPORT void GetModelPosterior(
    const std::pair<cv::Mat, cv::Mat>& currentCalibration,
    std::pair<cv::Mat, cv::Mat>& x,
    std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    );

/**
* \brief Computes MaxERE.
* \param intrinsic 3x3 intrinsic parameters
* \param distortion 1x4 distortion parameters
* \return MaxERE
*/
NIFTYCAL_WINEXPORT double ComputeMaxERE(
    const cv::Mat& intrinsic,
    const cv::Mat& distortion
    );

} // end namespace

#endif
