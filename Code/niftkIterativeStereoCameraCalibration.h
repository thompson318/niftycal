/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIterativeStereoCameraCalibration_h
#define niftkIterativeStereoCameraCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include "niftkIPoint2DDetector.h"
#include <list>
#include <memory>

namespace niftk
{

/**
* \file niftkIterativeStereoCameraCalibration.h
* \brief Performs iterative stereo camera calibration,
* as an extension of <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474">Datta 2009</a>.
* \param optimise3D if true and ITK is compiled in, will additionally optimise all
* camera parameters by minimising the RMS reconstruction error, reconstructing the target points in 3D.
* \return rms re-projection and 3D reconstruction error
*
* \ingroup calibration
*/
NIFTYCAL_WINEXPORT cv::Matx21d IterativeStereoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesLeft,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesRight,
    const cv::Size2i& imageSize,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImagesLeft,
    cv::Mat& intrinsicLeft,
    cv::Mat& distortionLeft,
    std::vector<cv::Mat>& rvecsLeft,
    std::vector<cv::Mat>& tvecsLeft,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImagesRight,
    cv::Mat& intrinsicRight,
    cv::Mat& distortionRight,
    std::vector<cv::Mat>& rvecsRight,
    std::vector<cv::Mat>& tvecsRight,
    cv::Mat& leftToRightRotationMatrix,
    cv::Mat& leftToRightTranslationVector,
    cv::Mat& essentialMatrix,
    cv::Mat& fundamentalMatrix,
    const int& cvFlags = 0,
    const bool& optimise3D = false // only if true AND ITK is compiled in.
    );

} // end namespace

#endif
