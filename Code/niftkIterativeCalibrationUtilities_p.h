/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIterativeCalibrationUtilities_p_h
#define niftkIterativeCalibrationUtilities_p_h

#include "niftkTypes.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <memory>
#include <cv.h>

namespace niftk
{

/**
* \file niftkIterativeCalibrationUtilities_p.h
* \brief Private (as in 'deliberately not exported') utility functions.
*/

void ExtractTwoCopiesOfControlPoints(
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& list,
    std::list<PointSet>& a,
    std::list<PointSet>& b);

void ExtractDistortedControlPoints(
    const std::pair< cv::Mat, niftk::PointSet>& referenceData,
    const cv::Mat& intrinsic,
    const cv::Mat& distortion,
    const cv::Mat& originalImage,
    std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat>& outputDetectorAndImage,
    PointSet& outputPoints
    );

} // end namespace

#endif
