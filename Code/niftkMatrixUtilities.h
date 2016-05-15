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
* \brief Various matrix related functions.
*/
namespace niftk
{

NIFTYCAL_WINEXPORT cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix3x3,
                                                              const cv::Mat& translationVector1x3);

NIFTYCAL_WINEXPORT cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector1x3,
                                                 const cv::Mat& translationVector1x3);


/**
* \brief Averages a list of rigid body matrices.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use lists and cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d AverageMatricesUsingEigenValues(const std::list<cv::Matx44d >&);


/**
* \brief Calculates Hand-Eye by matrix multiplication.
*
* Rather than have a tracked calibration object, we assume here that
* the caller can provide a modelToTrackerTransform to convert model
* points to tracker points. So, for example, the user has a stationary
* chessboard, and uses point based registration to register model coordinates
* to the tracker coordinates.
*
* \param modelToTrackerTransform transform to convert model coordinates into tracker coordinates.
* \param handMatrices matrices, synchronised with eyeMatrices, describing how the hand (tracker) moves.
* \param eyeMatrices matrices, synchronised with handMatrices, describing how the eye (camera) moves.
* \return cv::Matx44d hand-eye matrix
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeByDirectMatrixMultiplication(
    const cv::Matx44d&             modelToTrackerTransform,
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices
    );

} // end namespace

#endif
