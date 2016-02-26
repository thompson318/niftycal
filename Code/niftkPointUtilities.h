/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkPointUtilities_h
#define niftkPointUtilities_h

#include "niftkWin32ExportHeader.h"
#include "niftkTypes.h"

/**
* \file niftkPointUtilities.h
* \brief Defines various simple functions to process points.
*/
namespace niftk
{

/**
* \brief Creates a new copy of the input list.
*/
NIFTYCAL_WINEXPORT PointSet CopyPoints(const PointSet& p);


/**
* \brief Rescales by multiplying each point by the scale factor.
* \param scaleFactor contains a multiplier for x,y.
*/
NIFTYCAL_WINEXPORT PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor);


/**
* \brief Converts PointSet to vector for many OpenCV functions.
*/
NIFTYCAL_WINEXPORT void ConvertPoints(const PointSet& input,
                                      std::vector<cv::Point2f>& outputPoint,
                                      std::vector<niftk::IdType>& outputId
                                     );


/**
* \brief Converts PointSet to vector for many OpenCV functions.
*/
NIFTYCAL_WINEXPORT void ConvertPoints(const std::vector<cv::Point2f>& inputPoint,
                                      const std::vector<niftk::IdType>& inputId,
                                      PointSet& output
                                     );

/**
* \brief Extracts the common (same identifier) points in A and B.
*/
NIFTYCAL_WINEXPORT void ExtractCommonPoints(const PointSet& inputA,
                                            const PointSet& inputB,
                                            std::vector<cv::Point2f>& outputA,
                                            std::vector<cv::Point2f>& outputB
                                           );


/**
* \brief Computes the RMS error between common (same identifier) points in A and B.
* \throw if no common points.
*/
NIFTYCAL_WINEXPORT double ComputeRMSDifferenceBetweenMatchingPoints(const PointSet& inputA,
                                                                    const PointSet& inputB
                                                                   );


/**
* \brief Maps all points in distortedPoints to undistortedPoints.
*/
NIFTYCAL_WINEXPORT void UndistortPoints(const PointSet& distortedPoints,
                                        const cv::Mat& cameraIntrinsics,
                                        const cv::Mat& distortionCoefficients,
                                        PointSet& undistortedPoints
                                       );


/**
* \brief Maps all points in undistortedPoints to distortedPoints.
*/
NIFTYCAL_WINEXPORT void DistortPoints(const PointSet& undistortedPoints,
                                      const cv::Mat& cameraIntrinsics,
                                      const cv::Mat& distortionCoefficients,
                                      PointSet& distortedPoints
                                     );

} // end namespace

#endif
