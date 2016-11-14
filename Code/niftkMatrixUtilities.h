/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkMatrixUtilities_h
#define niftkMatrixUtilities_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <list>

/**
* \file niftkMatrixUtilities.h
* \brief Utilities for various matrix based functions.
*/
namespace niftk
{

NIFTYCAL_WINEXPORT cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix3x3,
                                                              const cv::Mat& translationVector1x3);

NIFTYCAL_WINEXPORT cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector1x3,
                                                 const cv::Mat& translationVector1x3);


NIFTYCAL_WINEXPORT void MatrixToRodrigues(const cv::Matx44d& mat,
                                          cv::Mat& rotationVector1x3,
                                          cv::Mat& translationVector1x3);

NIFTYCAL_WINEXPORT cv::Matx14d RodriguesToAxisAngle(const cv::Mat& rotationVector1x3);

/**
* \brief Converts a list to a vector, in order, enabling you to set a limit to how many you convert.
*/
NIFTYCAL_WINEXPORT std::vector<cv::Matx44d> MatrixListToVector(const std::list<cv::Matx44d>& matrices,
                                                               const unsigned int& maximumSize);


/**
* \brief Averages a list of rigid body matrices.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use lists and cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d AverageMatricesUsingEigenValues(const std::list<cv::Matx44d >&);


/**
* \brief Given a hand-eye matrix, and tracking matrices, gives an average
* estimate of the modelToWorld transform.
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateAverageModelToWorld(
    const cv::Matx44d&             handEyeMatrix,
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices
    );


/**
* \brief Given a 3x3 matrix, where the middle entry is the
* maximum of that neighbourhood, will fit a quadratic surface
* and then find the maximum, for sub-pixel resolution.
*
* Assumes the middle entry is coordinate (0, 0), so the
* outputPoint is a (dx, dy) from the centre.
*/
NIFTYCAL_WINEXPORT void InterpolateMaximumOfQuadraticSurface(const cv::Matx33d& matrix,
                                                             cv::Point2d& outputPoint
                                                            );

} // end namespace

#endif
