/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkHandEyeCalibration_h
#define niftkHandEyeCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkMatrixUtilities.h"
#include <cv.h>
#include <list>

/**
* \file niftkHandEyeCalibration.h
* \brief Entry point for a range of hand-eye calibration techniques.
*/
namespace niftk
{

/**
* \brief Calculates Hand-Eye by matrix multiplication.
*
* The eyeMatrices are a transformation matrix from model coordinates
* (e.g. points on a chessboard), to camera coordinates.
* The modelToTrackerTransform puts
* these model points into tracker space. The handMatrices are
* from tracking marker (hand) to tracker space. So, the handEye
* calibration is simply tracking marker (hand) to tracker,
* tracker to model, model to camera (eye).
*
* If handMatrices and eyeMatrices are lists of length > 1, then the
* hand-eye is calculated for each pair, and then the matrix average taken.
* \see niftk::AverageMatricesUsingEigenValues.
*
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeByDirectMatrixMultiplication(
    const cv::Matx44d&            modelToTrackerTransform,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices
    );


/**
* \brief Calculates Hand-Eye by <a href="http://dx.doi.org/10.1109/70.34770">Tsai's 1989 method</a>.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use lists and cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    double&                       residualRotation,
    double&                       residualTranslation
    );


} // end namespace

#endif
