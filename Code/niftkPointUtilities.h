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
#include "niftkNiftyCalTypes.h"
#include <list>
#include <random>

/**
* \file niftkPointUtilities.h
* \brief Utilities to process points.
*/
namespace niftk
{

/**
* \brief Computes the Euclidean distance between two points a and b.
*/
NIFTYCAL_WINEXPORT double DistanceBetween(const cv::Point3d& a, const cv::Point3d& b);

/**
* \brief Computes the Euclidean distance between two points a and b.
*/
NIFTYCAL_WINEXPORT double DistanceBetween(const cv::Point2d& a, const cv::Point2d& b);

/**
* \brief Creates a new copy of the input list.
*/
NIFTYCAL_WINEXPORT PointSet CopyPoints(const PointSet& p);

/**
* \brief Clears b and copies from a to b.
*/
NIFTYCAL_WINEXPORT void CopyPointsInto(const PointSet& a, PointSet& b);

/**
* \brief Rescales by multiplying each point by the scale factor.
* \param scaleFactor contains a multiplier for x and y.
*/
NIFTYCAL_WINEXPORT PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor);

/**
* \brief Converts PointSet to vector for many OpenCV functions.
*/
NIFTYCAL_WINEXPORT void ConvertPoints(const PointSet& input,
                                      std::vector<cv::Point2f>& outputPoint,
                                      std::vector<niftk::NiftyCalIdType>& outputId
                                     );

/**
* \brief Converts vector of points to our PointSet data type.
*/
NIFTYCAL_WINEXPORT void ConvertPoints(const std::vector<cv::Point2f>& inputPoint,
                                      const std::vector<niftk::NiftyCalIdType>& inputId,
                                      PointSet& output
                                     );

/**
* \brief Extracts the common (same identifier) points in A and B.
* \return outputA and outputB are the same length and ordered.
*/
NIFTYCAL_WINEXPORT void ExtractCommonPoints(const PointSet& inputA,
                                            const PointSet& inputB,
                                            std::vector<cv::Point2f>& outputA,
                                            std::vector<cv::Point2f>& outputB,
                                            std::vector<niftk::NiftyCalIdType>& commonIds
                                           );

/**
* \brief Extracts the common (same identifier) points in A and B.
* \return outputA and outputB are the same length and ordered.
*/
NIFTYCAL_WINEXPORT void ExtractCommonPoints(const Model3D& inputA,
                                            const Model3D& inputB,
                                            std::vector<cv::Point3d>& outputA,
                                            std::vector<cv::Point3d>& outputB,
                                            std::vector<niftk::NiftyCalIdType>& commonIds
                                           );

NIFTYCAL_WINEXPORT void ExtractCommonPoints(const Model3D& inputA,
                                            const PointSet& inputB,
                                            std::vector<cv::Point3d>& outputA,
                                            std::vector<cv::Point2d>& outputB,
                                            std::vector<niftk::NiftyCalIdType>& commonIds
                                           );

/**
* \brief Computes the RMS error between common (same identifier) points in a and b.
* \throw if no common points.
*/
NIFTYCAL_WINEXPORT double ComputeRMSDifferenceBetweenMatchingPoints(const PointSet& a,
                                                                    const PointSet& b,
                                                                    cv::Point2d& sumSquaredError,
                                                                    cv::Point2d& rmsForEachAxis
                                                                   );

/**
* \brief Computes the RMS error between common (same identifier) points in a and b.
* \throw if no common points.
*/
NIFTYCAL_WINEXPORT double ComputeRMSDifferenceBetweenMatchingPoints(const Model3D& a,
                                                                    const Model3D& b,
                                                                    cv::Point3d& sumSquaredError,
                                                                    cv::Point3d& rmsForEachAxis
                                                                   );

/**
* \brief Method to check if a PointSet contains sub-pixel coordinates.
*/
NIFTYCAL_WINEXPORT bool PointSetContainsNonIntegerPositions(const PointSet& p);

/**
* \brief Checks that a and b contain the same number of points, with matching
* identifiers, and the 3D location of these points is within some tolerance (i.e Eucliden distance).
*/
NIFTYCAL_WINEXPORT bool MatchesToWithinTolerance(const PointSet& a, const PointSet& b, const double& tolerance);

/**
* \brief Maps all points in distortedPoints (i.e. observed) to undistortedPoints (i.e. corrected).
*/
NIFTYCAL_WINEXPORT void UndistortPoints(const PointSet& distortedPoints,
                                        const cv::Mat& cameraIntrinsics,
                                        const cv::Mat& distortionCoefficients,
                                        PointSet& undistortedPoints
                                       );

/**
* \brief Maps all points in distortedPoints (i.e. observed) to undistortedPoints (i.e. corrected).
*/
NIFTYCAL_WINEXPORT void UndistortPoints(const std::vector<PointSet>& distortedPoints,
                                        const cv::Mat& cameraIntrinsics,
                                        const cv::Mat& distortionCoefficients,
                                        std::vector<PointSet>& undistortedPoints
                                       );

/**
* \brief Maps all points in undistortedPoints (i.e. corrected) to distortedPoints (i.e. observed).
*/
NIFTYCAL_WINEXPORT void DistortPoints(const PointSet& undistortedPoints,
                                      const cv::Mat& cameraIntrinsics,
                                      const cv::Mat& distortionCoefficients,
                                      PointSet& distortedPoints
                                     );

/**
* \brief Maps all points in undistortedPoints (i.e. corrected) to distortedPoints (i.e. observed).
*/
NIFTYCAL_WINEXPORT void DistortPoints(const std::vector<PointSet>& undistortedPoints,
                                      const cv::Mat& cameraIntrinsics,
                                      const cv::Mat& distortionCoefficients,
                                      std::vector<PointSet>& distortedPoints
                                     );

/**
* \brief Compares input with reference, and keeps the closest matching points,
* determined by a percentage. (i.e. for trimmed least squares).
* \param percentage number between 0 and 1.
*/
NIFTYCAL_WINEXPORT PointSet TrimPoints(const PointSet& input,
                                       const PointSet& reference,
                                       const float& percentage
                                       );

/**
* \brief For points in left image, draws epipolar lines on right image.
*
* This takes care of undistortion. So, you pass in distorted points, and
* distorted images. The epi-polar lines are drawn on an undistorted image.
*
* \param whichImage set to 1 if you are passing in left hand points to
* draw on right image, or 2 if you swap left for right.
*/
NIFTYCAL_WINEXPORT cv::Mat DrawEpiLines(const PointSet& leftDistortedPoints,
                                        const cv::Mat& leftIntrinsics,
                                        const cv::Mat& leftDistortion,
                                        const int& whichImage,
                                        const cv::Mat& fundamentalMatrix,
                                        const cv::Mat& rightDistortedGreyImage,
                                        const cv::Mat& rightIntrinsics,
                                        const cv::Mat& rightDistortion
                                       );

/**
* \brief Transforms the inputModel by the given matrix.
*/
NIFTYCAL_WINEXPORT Model3D TransformModel(const Model3D& inputModel, const cv::Matx44d& matrix);

/**
* \brief Adds noise to point locations.
*/
NIFTYCAL_WINEXPORT PointSet AddGaussianNoise(std::default_random_engine& engine,
                                             std::normal_distribution<double>& normalDistribution,
                                             const PointSet& points
                                            );

/**
* \brief Used to project 3D model points to 2D, but only for points that exist
* in the given PointSet.
*/
NIFTYCAL_WINEXPORT unsigned int ProjectMatchingPoints(const Model3D& model,
                                                      const PointSet& points,
                                                      const cv::Matx44d& extrinsic,
                                                      const cv::Mat& intrinsic,
                                                      const cv::Mat& distortion,
                                                      std::vector<cv::Point2f>& observed,
                                                      std::vector<cv::Point2f>& projected,
                                                      std::vector<niftk::NiftyCalIdType>& ids
                                                     );

/**
* \brief Triangulates common (same identifier) points in left and right views.
*/
NIFTYCAL_WINEXPORT void TriangulatePointPairs(const PointSet& leftDistortedPoints,
                                              const PointSet& rightDistortedPoints,
                                              const cv::Mat& leftIntrinsics,
                                              const cv::Mat& leftDistortionParams,
                                              const cv::Mat& leftCameraRotationVector,
                                              const cv::Mat& leftCameraTranslationVector,
                                              const cv::Mat& leftToRightRotationMatrix,
                                              const cv::Mat& leftToRightTranslationVector,
                                              const cv::Mat& rightIntrinsics,
                                              const cv::Mat& rightDistortionParams,
                                              Model3D& outputTriangulatedPoints
                                             );

/**
* \brief Computes stereo RMS reconstruction error, using triangulation, via camera matrices.
*
* This takes the 2D camera points, in left and right hand camera, triangulates to a 3D
* position in the left hand camera space, multiplies by the inverse of each
* left camera intrinsic matrix, and compares to the model position.
*/
NIFTYCAL_WINEXPORT double ComputeRMSReconstructionError(const Model3D& model,
                                                        const std::list<PointSet>& listOfLeftHandPointSets,
                                                        const std::list<PointSet>& listOfRightHandPointSets,
                                                        const cv::Mat& leftIntrinsics,
                                                        const cv::Mat& leftDistortionParams,
                                                        const std::vector<cv::Mat>& rvecsLeft,
                                                        const std::vector<cv::Mat>& tvecsLeft,
                                                        const cv::Mat& rightIntrinsics,
                                                        const cv::Mat& rightDistortionParams,
                                                        const cv::Mat& leftToRightRotationMatrix,
                                                        const cv::Mat& leftToRightTranslationVector,
                                                        cv::Point3d& rmsForEachAxis
                                                       );

/**
* \brief Computes stereo RMS reconstruction error, using triangulation, via tracking matrices.
*
* This takes the 2D camera points, in left and right hand camera, triangulates to a 3D
* position in the left hand camera space, multiplies by the inverse of the hand-eye
* matrix, the inverse of the tracking matrix and the inverse of the model-to-world matrix.
*/
NIFTYCAL_WINEXPORT double ComputeRMSReconstructionError(const Model3D& model,
                                                        const std::list<PointSet>& listOfLeftHandPointSets,
                                                        const std::list<PointSet>& listOfRightHandPointSets,
                                                        const cv::Mat& leftIntrinsics,
                                                        const cv::Mat& leftDistortionParams,
                                                        const std::vector<cv::Mat>& rvecsLeft,
                                                        const std::vector<cv::Mat>& tvecsLeft,
                                                        const cv::Mat& rightIntrinsics,
                                                        const cv::Mat& rightDistortionParams,
                                                        const cv::Mat& leftToRightRotationMatrix,
                                                        const cv::Mat& leftToRightTranslationVector,
                                                        const std::list<cv::Matx44d>& trackingMatrices,
                                                        const cv::Matx44d& handEyeMatrix,
                                                        const cv::Matx44d& modelToWorldMatrix
                                                       );


/**
* \brief Computes mono reconstruction error, taking model points, and multiplying
* through camera, hand-eye, tracking and modelToWorld matrices.
*/
NIFTYCAL_WINEXPORT double ComputeRMSReconstructionError(const Model3D& model,
                                                        const std::list<PointSet>& pointSets,
                                                        const std::vector<cv::Mat>& rvecs,
                                                        const std::vector<cv::Mat>& tvecs,
                                                        const std::list<cv::Matx44d>& trackingMatrices,
                                                        const cv::Matx44d& handEyeMatrix,
                                                        const cv::Matx44d& modelToWorldMatrix
                                                       );

/**
* \brief Used to evaluate a stereo calibration's RMS re-projection error.
*/
NIFTYCAL_WINEXPORT double ComputeRMSReprojectionError(const Model3D& model,
                                                      const std::list<PointSet>& listOfLeftHandPointSets,
                                                      const std::list<PointSet>& listOfRightHandPointSets,
                                                      const cv::Mat& leftIntrinsics,
                                                      const cv::Mat& leftDistortionParams,
                                                      const std::vector<cv::Mat>& rvecsLeft,
                                                      const std::vector<cv::Mat>& tvecsLeft,
                                                      const cv::Mat& rightIntrinsics,
                                                      const cv::Mat& rightDistortionParams,
                                                      const cv::Mat& leftToRightRotationMatrix,
                                                      const cv::Mat& leftToRightTranslationVector
                                                     );

NIFTYCAL_WINEXPORT double ComputeRMSProjectionError(const Model3D& model,
                                                    const PointSet& points,
                                                    const cv::Mat& intrinsics,
                                                    const cv::Mat& distortionParams,
                                                    const cv::Mat& rvec,
                                                    const cv::Mat& tvec
                                                   );

NIFTYCAL_WINEXPORT unsigned long int GetNumberOfTriangulatablePoints(const Model3D& model,
                                                                     const std::list<PointSet>& listOfLeftHandPointSets,
                                                                     const std::list<PointSet>& listOfRightHandPointSets
                                                                    );


} // end namespace

#endif
