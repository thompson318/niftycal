/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkHomographyUtilities_h
#define niftkHomographyUtilities_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"

/**
* \file niftkHomographyUtilities.h
* \brief Utilities for mapping images between homographies.
*/
namespace niftk
{

/**
* \brief Maps all points in src to target via homography.
*/
NIFTYCAL_WINEXPORT void WarpPointsByHomography(const PointSet& src,
                                               const cv::Mat& homography,
                                               PointSet& target
                                              );


/**
* \brief Finds a homography using all matching (same identifier) in src and target.
* \throw if no common points.
*/
NIFTYCAL_WINEXPORT void FindHomography(const PointSet& src,
                                       const PointSet& target,
                                       cv::Mat& homography
                                      );


/**
* \brief Warps inputImage to outputImage, depending on
* homography that maps between distortedPoints and targetPoints.
*/
NIFTYCAL_WINEXPORT void WarpImageByCorrespondingPoints(const cv::Mat& inputImage,
                                                       const cv::Mat& cameraIntrinsics,
                                                       const cv::Mat& distortionCoefficients,
                                                       const PointSet& distortedPoints,
                                                       const PointSet& targetPoints,
                                                       const cv::Size2i outputImageSize,
                                                       cv::Mat& outputHomography,
                                                       cv::Mat& outputImage
                                                      );


/**
* \brief Warps inputImage to outputImage according on homography.
*/
NIFTYCAL_WINEXPORT void WarpImageByHomography(const cv::Mat& inputImage,
                                              const cv::Mat& homography,
                                              const cv::Size2i outputImageSize,
                                              cv::Mat& outputImage
                                             );

} // end namespace

#endif
