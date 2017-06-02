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
* in NifTK, but converted to cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    cv::Matx21d&                  residuals
    );


/**
* \brief Calculates Hand-Eye by doing non-linear part of
* <a href="http://dx.doi.org/10.1002/rcs.1478">Malti's 2013 method</a>.
* \param residual returns 2D rms reprojection error.
*
* Optimises intrinsic (4DOF), distortion (5DOF), model-to-world (6DOF) and hand-eye (6DOF).
*
* If ITK is not compiled in, this just sets residual to zero.
*/
NIFTYCAL_WINEXPORT void CalculateHandEyeUsingMaltisMethod(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    cv::Mat&                      intrinsic,
    cv::Mat&                      distortion,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    double&                       residual
    );


/**
* \brief Calculates Hand-Eye by non-linearly optimising extrinsic (tracker matrices),
* model-to-world and hand-eye matrices.
* \param residual returns 2D rms reprojection error.
*
* If ITK is not compiled in, this just sets residual to zero.
*/
NIFTYCAL_WINEXPORT void CalculateHandEyeByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const cv::Mat&                intrinsic,
    const cv::Mat&                distortion,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    double&                       residual
    );


/**
* \brief Calculates Hand-Eye by non-linearly optimising extrinsic (tracker matrices),
* model-to-world, hand-eye matrices and stereo left-to-right matrices.
* \param residuals returns 2D rms reprojection error if optimise3D is false
* or 3D rms reconstruction error if optimise3D is true.
*
* If ITK is not compiled in, this just sets residual to zero.
*/
NIFTYCAL_WINEXPORT void CalculateHandEyeInStereoByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    leftPointSets,
    const cv::Mat&                leftIntrinsic,
    const cv::Mat&                leftDistortion,
    const std::list<PointSet>&    rightPointSets,
    const cv::Mat&                rightIntrinsic,
    const cv::Mat&                rightDistortion,
    const std::list<cv::Matx44d>& handMatrices,
    const bool&                   optimise3D,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    cv::Matx44d&                  stereoExtrinsics,
    double&                       residual
    );


/**
* \brief Calculates Hand-Eye by Morgan et al. IPCAI 2017 paper.
*
* Note: The input are 3D points, and their corresponding 2D
* undistorted image points (i.e. 2D points after distortion
* correction has happened). Furthermore, the 3D points, are
* not model points, like points on a chessboard using the
* chessboard coordinate system. They are in fact in tracker
* space. As such, there is no need to have a regular grid
* of fiducials.
*
*/
NIFTYCAL_WINEXPORT void CalculateHandEyeUsingPoint2Line(
    const cv::Mat&                                           cameraMatrix,
    const std::vector<std::pair<cv::Point2d, cv::Point3d> >& pairedPoints,
    const double&                                            exitCondition,
    cv::Matx44d&                                             handEye
    );

} // end namespace

#endif
