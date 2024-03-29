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
#include <list>
#include <cv.h>

#ifdef NIFTYCAL_WITH_ITK
#include <itkMultipleValuedCostFunction.h>
#endif

namespace niftk
{

/**
* \brief Returns -1 if x < 0 and +1 otherwise.
*/
int Signum(const double& x);

/**
* \file niftkCalibrationUtilities_p.h
* \brief Private (as in 'deliberately not exported') utility functions.
*/

#ifdef NIFTYCAL_WITH_ITK

/**
* \brief For a single view, calculates projection errors.
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
* Then sets of 6 DOF for the extrinsic parameters of each view
*   p[9]  = r1 (Rodrigues)
*   p[10] = r2 (Rodrigues)
*   p[11] = r3 (Rodrigues)
*   p[12] = tx
*   p[13] = ty
*   p[14] = tz
* etc.
*
* \endverbatim
*/
void ComputeMonoProjectionErrors(const Model3D* const model,
                                 const std::list<PointSet>* const points,
                                 const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                 itk::MultipleValuedCostFunction::MeasureType& errors
                                );


/**
* \brief For a single pair of views, calculates projection error.
*/
void ComputeStereoProjectionErrors(const Model3D& model,
                                   const PointSet& leftPoints,
                                   const PointSet& rightPoints,
                                   const cv::Matx44d& leftExtrinsic,
                                   const cv::Mat& leftIntrinsic,
                                   const cv::Mat& leftDistortion,
                                   const cv::Matx44d& rightExtrinsic,
                                   const cv::Mat& rightIntrinsic,
                                   const cv::Mat& rightDistortion,
                                   itk::MultipleValuedCostFunction::MeasureType& errors
                                  );


/**
* \brief Used in stereo CostFunctions to compute the projection error over a number of views.
* \param parameters array of the format:
*
* \verbatim
*   p[0] = left fx
*   p[1] = left fy
*   p[2] = left cx
*   p[3] = left cy
*   p[4] = left d1
*   p[5] = left d2
*   p[6] = left d3
*   p[7] = left d4
*   p[8] = left d5
*   p[9] = right fx
*   p[10] = right fy
*   p[11] = right cx
*   p[12] = right cy
*   p[13] = right d1
*   p[14] = right d2
*   p[15] = right d3
*   p[16] = right d4
*   p[17] = right d5
*
* Then 6 DOF for left-to-right extrinsics:
*   p[18] = r1 (Rodrigues)
*   p[19] = r2 (Rodrigues)
*   p[20] = r3 (Rodrigues)
*   p[21] = tx
*   p[22] = ty
*   p[23] = tz
*
*
* Then sets of 6 DOF for left camera extrinsics for each view:
*   p[24] = r1 (Rodrigues)
*   p[25] = r2 (Rodrigues)
*   p[26] = r3 (Rodrigues)
*   p[27] = tx
*   p[28] = ty
*   p[29] = tz
* etc.
*
* \endverbatim
* \param errors which must be the right size when it is passed in
*/
void ComputeStereoProjectionErrors(const Model3D* const model,
                                   const std::list<PointSet>* const leftPoints,
                                   const std::list<PointSet>* const rightPoints,
                                   const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                   itk::MultipleValuedCostFunction::MeasureType& errors
                                  );


/**
* \brief For a single pair of views, calculates stereo reconstruction error.
*/
void ComputeStereoReconstructionErrors(const Model3D& model,
                                       const PointSet& leftPoints,
                                       const PointSet& rightPoints,
                                       const cv::Matx44d& leftExtrinsic,
                                       const cv::Mat& leftIntrinsic,
                                       const cv::Mat& leftDistortion,
                                       const cv::Mat& leftToRightRotationMatrix,
                                       const cv::Mat& leftToRightTranslationVector,
                                       const cv::Mat& rightIntrinsic,
                                       const cv::Mat& rightDistortion,
                                       itk::MultipleValuedCostFunction::MeasureType& errors
                                      );


/**
* \brief Computes stereo reconstruction errors, using the same
* format input parameters as ComputeStereoProjectionErrors.
* \param errors which must be the right size when it is passed in
*/
void ComputeStereoReconstructionErrors(const Model3D* const model,
                                       const std::list<PointSet>* const leftPoints,
                                       const std::list<PointSet>* const rightPoints,
                                       const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                       itk::MultipleValuedCostFunction::MeasureType& errors
                                      );

/**
 * \brief Computes distances from epi-polar lines to use as a registration metric.
 *
 * The parameters below say 'left' and 'right' as for each left point, you compute
 * the epi-polar line on the right hand image, and compare against the right hand point.
 * You can swap left/right to get symmetric error.
 */
void ComputeEpipolarErrors(const PointSet& leftPoints,
                           const cv::Mat& leftIntrinsic,
                           const cv::Mat& leftDistortion,
                           const PointSet& rightPoints,
                           const cv::Mat& rightIntrinsic,
                           const cv::Mat& rightDistortion,
                           const int& whichImage,
                           const cv::Mat& fundamentalMatrix,
                           itk::MultipleValuedCostFunction::MeasureType& errors
                          );

#endif

} // end namespace

#endif
