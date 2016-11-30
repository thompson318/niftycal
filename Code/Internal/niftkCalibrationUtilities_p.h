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
#include <itkMultipleValuedCostFunction.h>
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
* by taking the left extrinsics, and the left-to-right transformation
* and computing the corresponding right extrinsics.
*/
void ComputeStereoExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                             const std::vector<cv::Mat>& tvecsLeft,
                             const cv::Mat& leftToRightRotationMatrix,
                             const cv::Mat& leftToRightTranslationVector,
                             std::vector<cv::Mat>& rvecsRight,
                             std::vector<cv::Mat>& tvecsRight
                            );


/**
* \brief For a single view, calculates reprojection errors.
*/
void ComputeMonoProjectionErrors(const niftk::Model3D& model,
                                 const niftk::PointSet& points,
                                 const cv::Matx44d& extrinsic,
                                 const cv::Mat& intrinsic,
                                 const cv::Mat& distortion,
                                 itk::MultipleValuedCostFunction::MeasureType& errorValues
                                );


/**
* \brief Used in mono CostFunctions to compute the projection error over a number of views.
* \param parameters array of the format:
*
* \verbatim
*   p[0] = fx
*   p[1] = fy
*   p[2] = cx
*   p[3] = cy
*   p[4] = d1
*   p[5] = d2
*   p[6] = d3
*   p[7] = d4
*   p[8] = d5
*
* Then sets of 6 DOF for each view
*   p[9]  = r1 (Rodrigues)
*   p[10] = r2 (Rodrigues)
*   p[11] = r3 (Rodrigues)
*   p[12] = tx
*   p[13] = ty
*   p[14] = tz
* etc.
*/
void ComputeMonoProjectionErrors(const Model3D* const model,
                                 const std::list<PointSet>* const points,
                                 const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                 itk::MultipleValuedCostFunction::MeasureType& errors
                                );

} // end namespace

#endif
