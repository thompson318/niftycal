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
#include "niftkIPoint2DDetector.h"
#include <cv.h>

/**
* \file niftkHomographyUtilities.h
* \brief Utilities for mapping images between homographies.
*/
namespace niftk
{


/**
* \brief Warps inputImage to outputImage, depending on
* homography that maps between sourcePoints and targetPoints
*/
NIFTYCAL_WINEXPORT void WarpImageByCorrespondingPoints(const cv::Mat& inputImage,
                                                       const PointSet& sourcePoints,
                                                       const PointSet& targetPoints,
                                                       const cv::Size2i outputImageSize,
                                                       cv::Mat& outputImage
                                                      );

/**
* \brief Warps inputImage to outputImage, depending on homography.
*/
NIFTYCAL_WINEXPORT void WarpImageByHomography(const cv::Mat& inputImage,
                                              const cv::Mat& homography,
                                              const cv::Size2i outputImageSize,
                                              cv::Mat& outputImage
                                             );

} // end namespace

#endif
