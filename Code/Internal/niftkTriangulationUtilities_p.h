/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTriangulationUtilities_h
#define niftkTriangulationUtilities_h

#include "niftkNiftyCalTypes.h"
#include <list>
#include <vector>

/**
* \file niftkTriangulationUtilities.h
* \brief Utilities to triangulate points.
*/
namespace niftk
{

/**
* \brief Triangulates a vector of undistorted (i.e. already correction for distortion) 2D point pairs back into 3D.
*
* From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997.
*
* and
*
* <a href="http://www.morethantechnical.com/2012/01/04/
* simple-triangulation-with-opencv-from-harley-zisserman-w-code/">here</a>.
*
* and
*
* Price 2012, Computer Vision: Models, Learning and Inference.
*
* \param leftCameraUndistortedPoints left camera undistorted points
* \param leftCameraUndistortedPoints right camera undistorted points
* \param leftCameraIntrinsicParams [3x3] matrix for left camera.
* \param leftCameraRotationVector [1x3] matrix for the left camera rotation vector.
* \param leftCameraTranslationVector [1x3] matrix for the left camera translation vector.
* \param rightCameraIntrinsicParams [3x3] matrix for right camera.
* \param rightCameraRotationVector [1x3] matrix for the right camera rotation vector.
* \param rightCameraTranslationVector [1x3] matrix for right camera translation vector.
* \param outputPoints reconstructed 3D points, w.r.t. left camera (ie. in left camera coordinates).
*/
void TriangulatePointPairs(
  const std::vector<cv::Point2f>& leftCameraUndistortedPoints,
  const std::vector<cv::Point2f>& rightCameraUndistortedPoints,
  const cv::Mat& leftCameraIntrinsicParams,
  const cv::Mat& leftCameraRotationVector,
  const cv::Mat& leftCameraTranslationVector,
  const cv::Mat& rightCameraIntrinsicParams,
  const cv::Mat& rightCameraRotationVector,
  const cv::Mat& rightCameraTranslationVector,
  std::vector<cv::Point3f>& outputTriangulatedPoints
  );

} // end namespace

#endif
