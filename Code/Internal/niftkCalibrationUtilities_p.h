/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkCalibrationUtilities_p_h
#define niftkCalibrationUtilities_p_h

#include "niftkNiftyCalTypes.h"
#include <list>
#include <cv.h>

namespace niftk
{

/**
* \file niftkCalibrationUtilities_p.h
* \brief Private (as in 'deliberately not exported') utility functions.
*/

/**
* \brief Computes a consistent set of left and right extrinsics,
* by taking the left extrinsics, and the right-to-left transformation
* and computing the corresponding right extrinsics.
*/
void ComputeStereoExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                             const std::vector<cv::Mat>& tvecsLeft,
                             const cv::Mat& leftToRightRotationMatrix,
                             const cv::Mat& leftToRightTranslationVector,
                             std::vector<cv::Mat>& rvecsRight,
                             std::vector<cv::Mat>& tvecsRight
                            );

} // end namespace

#endif
