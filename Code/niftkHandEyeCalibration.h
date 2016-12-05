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
#include "niftkNiftyCalTypes.h"
#include <cv.h>
#include <list>

/**
* \file niftkHandEyeCalibration.h
* \brief Entry point for a range of hand-eye calibration techniques.
* \ingroup calibration
*/
namespace niftk
{

/**
* \brief Calculates Hand-Eye by matrix multiplication.
*
* The eyeMatrices are a transformation matrix from model coordinates
* (e.g. points on a chessboard), to camera (eye) coordinates.
* The modelToTrackerTransform puts
* these model points into tracker space. The handMatrices are
* from tracking marker (hand) to tracker space. So, the handEye
* calibration is simply the composition of tracking marker (hand)
* to tracker, tracker to model then model to camera (eye).
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
* \param residuals returns residual rotation and residual translation.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use lists and cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    cv::Matx21d&                  residuals
    );


#ifdef NIFTYCAL_WITH_ITK

/**
* \brief Calculates Hand-Eye based on something akin to <a href="http://dx.doi.org/10.1002/rcs.1478">Malti's 2013 method</a>.
* \param residual returns 2D rms reprojection error.
*
* This does an initial hand-eye using Tsai's 1989 linear approximation, followed by
* optimisation of the intrinsic (4DOF), distortion (5DOF), model-to-world (6DOF) and hand-eye (6DOF).
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeUsingMaltisMethod(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    cv::Mat&                      intrinsic,
    cv::Mat&                      distortion,
    double&                       residual
    );


/**
* \brief Calculates Hand-Eye by optimising extrinsic (tracker matrices), model-to-world and hand-eye matrices.
* \param residual returns 2D rms reprojection error.
*
* This does an initial hand-eye using Tsai's 1989 linear approximation, followed by
* optimisation of the extrinsic (6N DOF), model-to-world (6DOF) and hand-eye (6DOF).
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    const cv::Mat&                intrinsic,
    const cv::Mat&                distortion,
    double&                       residual
    );


/**
* \brief Calculates Hand-Eye by optimising extrinsic (tracker matrices), model-to-world, hand-eye matrices and stereo left-to-right matrices.
* \param residuals returns 2D rms reprojection error and 3D rms reconstruction error.
*
* This does an initial hand-eye using Tsai's 1989 linear approximation, followed by
* optimisation of the extrinsic (6N DOF), model-to-world (6DOF), hand-eye (6DOF) and left-to-right (6DOF).
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeInStereoByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    leftPointSets,
    const cv::Mat&                leftIntrinsic,
    const cv::Mat&                leftDistortion,
    const std::list<PointSet>&    rightPointSets,
    const cv::Mat&                rightIntrinsic,
    const cv::Mat&                rightDistortion,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    const bool&                   optimise3D,
    cv::Matx44d&                  stereoExtrinsics,
    double&                       residual
    );

#endif // NIFTYCAL_WITH_ITK

} // end namespace

#endif
