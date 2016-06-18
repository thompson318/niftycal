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
* \brief Triangulates a vector of undistorted (i.e. already correction for distortion) 2D point pairs back into 3D.
*
* From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997.
*
* and
*
* <a href="http://www.morethantechnical.com/2012/01/04/
* simple-triangulation-with-opencv-from-harley-zisserman-w-code/">here</a>.
*
* and
*
* Price 2012, Computer Vision: Models, Learning and Inference.
*
* \param leftCameraUndistortedPoints left camera undistorted points
* \param leftCameraUndistortedPoints right camera undistorted points
* \param leftCameraIntrinsicParams [3x3] matrix for left camera.
* \param leftCameraRotationVector [1x3] matrix for the left camera rotation vector.
* \param leftCameraTranslationVector [1x3] matrix for the left camera translation vector.
* \param rightCameraIntrinsicParams [3x3] matrix for right camera.
* \param rightCameraRotationVector [1x3] matrix for the right camera rotation vector.
* \param rightCameraTranslationVector [1x3] matrix for right camera translation vector.
* \param outputPoints reconstructed 3D points, w.r.t. left camera (ie. in left camera coordinates).
*/
NIFTYCAL_WINEXPORT void TriangulatePointPairs(
  const std::vector<cv::Point2f>& leftCameraUndistortedPoints,
  const std::vector<cv::Point2f>& rightCameraUndistortedPoints,
  const cv::Mat& leftCameraIntrinsicParams,
  const cv::Mat& leftCameraRotationVector,
  const cv::Mat& leftCameraTranslationVector,
  const cv::Mat& rightCameraIntrinsicParams,
  const cv::Mat& rightCameraRotationVector,
  const cv::Mat& rightCameraTranslationVector,
  std::vector<cv::Point3f>& outputTriangulatedPoints
  );

/**
* \brief Overrides the above method.
*/
NIFTYCAL_WINEXPORT void TriangulatePointPairs(
  const PointSet& leftDistortedPoints,
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
* \brief Transforms the inputModel by the given matrix.
*/
NIFTYCAL_WINEXPORT Model3D TransformModel(
  const Model3D& inputModel,
  const cv::Matx44d& matrix
  );

/**
* \brief Adds noise to point locations.
*/
NIFTYCAL_WINEXPORT PointSet AddGaussianNoise(std::default_random_engine& engine,
                                             std::normal_distribution<double>& normalDistribution,
                                             const PointSet& points
                                            );

/**
* \brief Used to evaluate a stereo calibration's RMS reconstruction error.
*/
NIFTYCAL_WINEXPORT double ComputeRMSReconstructionError(
  const Model3D& model,
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

} // end namespace

#endif
