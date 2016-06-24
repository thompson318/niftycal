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
#include "niftkNiftyCalTypes.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <memory>

namespace niftk
{

/**
* \brief Given lists of detectors and images, performs iterative mono camera calibration,
* as seen in <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474>Dutta 2009</a>.
* \see niftk::MonoCameraCalibration
* \return rms re-projection error
*/
NIFTYCAL_WINEXPORT double IterativeMonoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
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
