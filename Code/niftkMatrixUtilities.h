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


NIFTYCAL_WINEXPORT void MatrixToRodrigues(const cv::Matx44d& mat,
                                          cv::Mat& rotationVector1x3,
                                          cv::Mat& translationVector1x3);

/**
* \brief Converts a list to a vector, enabling you to set a limit to how many you convert.
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


/**
* \brief Calculates Hand-Eye by <a href="http://dx.doi.org/10.1109/70.34770">Tsai's 1989 method</a>.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use lists and cv::Matx44d for NiftyCal.
*
* \param handMatrices matrices, synchronised with eyeMatrices, describing how the hand (tracker) moves.
* \param eyeMatrices matrices, synchronised with handMatrices, describing how the eye (camera) moves.
* \return cv::Matx44d hand-eye matrix
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices,
    double& residualRotation,
    double& residualTranslation
    );


/**
* \brief Given a hand-eye matrix, and tracking matrices, gives an average
* estimate of the modelToWorld transform.
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateAverageModelToWorld(
    const cv::Matx44d&             handEyeMatrix,
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices
    );

} // end namespace

#endif
