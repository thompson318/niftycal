/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIterativeMonoCameraCalibration_h
#define niftkIterativeMonoCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkTypes.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <memory>

namespace niftk
{

NIFTYCAL_WINEXPORT void ExtractTwoCopiesOfControlPoints(
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& list,
    std::list<PointSet>& a,
    std::list<PointSet>& b);

NIFTYCAL_WINEXPORT void ExtractDistortedControlPoints(
    const std::pair< cv::Size2i, niftk::PointSet>& referenceData,
    const cv::Mat& intrinsic,
    const cv::Mat& distortion,
    const cv::Mat& originalImage,
    std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat>& outputDetectorAndImage,
    PointSet& outputPoints
    );

/**
* \brief Given a list of images, performs mono camera
* calibration according to: Dutta ICCV 2009.
* \return rms re-projection error
*/
NIFTYCAL_WINEXPORT double IterativeMonoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Size2i, niftk::PointSet>& referenceImageData,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImages,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImages,
    cv::Size2i& imageSize,
    cv::Mat& intrinsic,
    cv::Mat& distortion,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs,
    const int& cvFlags = 0
    );

} // end namespace

#endif
