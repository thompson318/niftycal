/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeMonoCameraCalibration.h"
#include "niftkMonoCameraCalibration.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double IterativeMonoCameraCalibration(
    const Model3D& model,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImages,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImages,
    cv::Size2i& imageSize,
    cv::Mat& intrinsic,
    cv::Mat& distortion,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs
    )
{

  double rms = 0;
  std::list<PointSet> pointsFromImages;
  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator iter;

  // 1. Detect control points: Detect calibration pattern control
  // points (corners, circle or ring centers) in the input images.
  for (iter = detectorAndOriginalImages.begin(); iter != detectorAndOriginalImages.end(); ++iter)
  {
    PointSet points = (*iter).first->GetPoints();
    if (!points.empty())
    {
      pointsFromImages.push_back(points);
    }
  }

  // 2. Parameter Fitting: Use the detected control points to estimate
  // camera parameters using Levenberg-Marquardt.
  rms = niftk::MonoCameraCalibration(
        model,
        pointsFromImages,
        imageSize,
        intrinsic,
        distortion,
        rvecs,
        tvecs
        );

  return rms;
}

} // end namespace
