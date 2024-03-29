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
* \ingroup utilities
*/
namespace niftk
{

/**
\brief In debug mode at least, cv::Rodrigues can flip the 1x3 rotation vector to 3x1, so this function doesn't
*/
NIFTYCAL_WINEXPORT void SafeRodrigues(const cv::Mat& rotationMatrix3x3,
                                      cv::Mat& rotationVector1x3);

/**
* \brief Converts 3x3 rotation and 3x1 translation (i.e. stereo extrinsics) to a 4x4 matrix.
*/
NIFTYCAL_WINEXPORT cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix3x3,
                                                              const cv::Mat& translationVector3x1);


/**
* \brief Converts 1x3 rotation (i.e. Rodrigues parameters) and 1x3 translation to a 4x4 matrix.
*/
NIFTYCAL_WINEXPORT cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector1x3,
                                                 const cv::Mat& translationVector1x3);


/**
* \brief Converts 4x4 transformation to 1x3 rotation (i.e. Rodrigues parameters) and 1x3 translation.
*/
NIFTYCAL_WINEXPORT void MatrixToRodrigues(const cv::Matx44d& mat,
                                          cv::Mat& rotationVector1x3,
                                          cv::Mat& translationVector1x3);

/**
* \brief Convers 1x3 Rodrigues rotation parameters to Axis-Angle representation.
*/
NIFTYCAL_WINEXPORT cv::Matx14d RodriguesToAxisAngle(const cv::Mat& rotationVector1x3);


/**
* \brief Convers Axis-Angle representation to Rodrigues parameters.
*/
NIFTYCAL_WINEXPORT cv::Mat AxisAngleToRodrigues(const cv::Matx14d& axisAngle);


/**
* \brief Converts an angle in Rodrigues notation to Euler angles, lets say roll, pitch, yaw
*/
NIFTYCAL_WINEXPORT cv::Mat RodriguesToEulerAngles(const cv::Mat& rotationVector1x3);


/**
* \brief Converts a list of 4x4 matrices to a vector, in order, enabling you to set a limit to how many you convert.
*/
NIFTYCAL_WINEXPORT std::vector<cv::Matx44d> MatrixListToVector(const std::list<cv::Matx44d>& matrices,
                                                               const unsigned int& maximumSize);


/**
* \brief Averages a list of rigid body matrices.
*
* Originally implemented by Steve Thompson (s.thompson@ucl.ac.uk)
* in NifTK, but converted to use cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d AverageMatricesUsingEigenValues(const std::list<cv::Matx44d >&);


/**
* \brief Given ordered hand-eye matrices and corresponding tracking matrices,
* gives an estimate of the average modelToWorld transform.
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


/**
* \brief Simply returns [0|1|2] to indicate which axes has the biggest absolute value.
*/
NIFTYCAL_WINEXPORT int GetMajorAxisIndex(const cv::Vec3d& v);


/**
* \brief Calculates the left-to-right matrix.
*/
NIFTYCAL_WINEXPORT cv::Matx44d GetLeftToRightMatrix(const cv::Matx44d& leftExtrinsics,
                                                    const cv::Matx44d& rightExtrinsics);


/**
* \brief Calculates the left-to-right matrix.
*/
NIFTYCAL_WINEXPORT void GetLeftToRightMatrix(const cv::Mat& leftRVec1x3,
                                             const cv::Mat& leftTVec1x3,
                                             const cv::Mat& rightRVec1x3,
                                             const cv::Mat& rightTVec1x3,
                                             cv::Mat& leftToRightMatrix3x3,
                                             cv::Mat& leftToRightTVec3x1
                                             );


/**
 * \brief Computes F from Camera Calibration.
 */
NIFTYCAL_WINEXPORT cv::Mat ComputeFundamentalMatrixFromCameraCalibration(const cv::Mat& leftIntrinsic,
                                                                         const cv::Mat& leftToRightRotationMatrix,
                                                                         const cv::Mat& leftToRightTranslationVector,
                                                                         const cv::Mat& rightIntrinsic
                                                                        );

/**
* \brief Computes a consistent set of left and right extrinsics,
* by taking the left extrinsics, and the left-to-right transformation
* and computing the corresponding right extrinsics.
*/
NIFTYCAL_WINEXPORT void ComputeStereoExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                                                const std::vector<cv::Mat>& tvecsLeft,
                                                const cv::Mat& leftToRightRotationMatrix,
                                                const cv::Mat& leftToRightTranslationVector,
                                                std::vector<cv::Mat>& rvecsRight,
                                                std::vector<cv::Mat>& tvecsRight
                                               );

} // end namespace

#endif
