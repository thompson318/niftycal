/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingCameraCalibration.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double InternalRenderingMonoIntrinsicCameraCalibration(const std::string& modelFileName,
                                                       const std::string& textureFileName,
                                                       const std::vector<cv::Mat>& images,
                                                       const std::vector<cv::Mat>& rvecs,
                                                       const std::vector<cv::Mat>& tvecs,
                                                       const double& learningRate,
                                                       cv::Mat& intrinsic,
                                                       cv::Mat& distortion
                                                      )
{
  return 0;
}


//-----------------------------------------------------------------------------
double InternalRenderingMonoExtrinsicCameraCalibration(const std::string& modelFileName,
                                                       const std::string& textureFileName,
                                                       const std::vector<cv::Mat>& images,
                                                       const cv::Mat& intrinsic,
                                                       const cv::Mat& distortion,
                                                       const double& learningRate,
                                                       std::vector<cv::Mat>& rvecs,
                                                       std::vector<cv::Mat>& tvecs
                                                      )
{
  return 0;
}


//-----------------------------------------------------------------------------
double InternalRenderingStereoCameraCalibration(const std::string& modelFileName,
                                                const std::string& textureFileName,
                                                const std::vector<cv::Mat>& leftImages,
                                                const std::vector<cv::Mat>& rightImages,
                                                const cv::Mat& intrinsicLeft,
                                                const cv::Mat& distortionLeft,
                                                const cv::Mat& intrinsicRight,
                                                const cv::Mat& distortionRight,
                                                const double& learningRate,
                                                std::vector<cv::Mat>& rvecsLeft,
                                                std::vector<cv::Mat>& tvecsLeft,
                                                std::vector<cv::Mat>& rvecsRight,
                                                std::vector<cv::Mat>& tvecsRight,
                                                cv::Mat& leftToRightRotationMatrix,
                                                cv::Mat& leftToRightTranslationVector
                                               )
{
  return 0;
}


//-----------------------------------------------------------------------------
void RenderingMonoCameraCalibration(const std::string& modelFileName,
                                    const std::string& textureFileName,
                                    const std::vector<cv::Mat>& images,
                                    cv::Mat& intrinsic,
                                    cv::Mat& distortion,
                                    std::vector<cv::Mat>& rvecs,
                                    std::vector<cv::Mat>& tvecs
                                   )
{
  double learningRate = 1;

  do
  {

    double cost = std::numeric_limits<double>::min() + 1;
    double previousCost = std::numeric_limits<double>::min();
    unsigned int numberOfIterations = 0;

    do
    {

      cost = InternalRenderingMonoIntrinsicCameraCalibration(modelFileName,
                                                             textureFileName,
                                                             images,
                                                             rvecs,
                                                             tvecs,
                                                             learningRate,
                                                             intrinsic,
                                                             distortion
                                                             );

      std::cout << "RenderingMonoCameraCalibration:Cost=" << cost << ", Intrinsic="
                << intrinsic.at<double>(0, 0) << " "
                << intrinsic.at<double>(1, 1) << " "
                << intrinsic.at<double>(0, 2) << " "
                << intrinsic.at<double>(1, 2) << " "
                << distortion.at<double>(0, 0) << " "
                << distortion.at<double>(0, 1) << " "
                << distortion.at<double>(0, 2) << " "
                << distortion.at<double>(0, 3)
                << std::endl;

      cost = InternalRenderingMonoExtrinsicCameraCalibration(modelFileName,
                                                             textureFileName,
                                                             images,
                                                             intrinsic,
                                                             distortion,
                                                             learningRate,
                                                             rvecs,
                                                             tvecs
                                                             );

      std::cout << "RenderingMonoCameraCalibration:Cost=" << cost << std::endl;

      numberOfIterations++;

    } while (cost - previousCost > 0.0001 && numberOfIterations < 100);

    learningRate /= 2.0;

  } while (learningRate > 0.1);
}


//-----------------------------------------------------------------------------
void RenderingStereoCameraCalibration(const std::string& modelFileName,
                                      const std::string& textureFileName,
                                      const std::vector<cv::Mat>& leftImages,
                                      const std::vector<cv::Mat>& rightImages,
                                      cv::Mat& intrinsicLeft,
                                      cv::Mat& distortionLeft,
                                      std::vector<cv::Mat>& rvecsLeft,
                                      std::vector<cv::Mat>& tvecsLeft,
                                      cv::Mat& intrinsicRight,
                                      cv::Mat& distortionRight,
                                      std::vector<cv::Mat>& rvecsRight,
                                      std::vector<cv::Mat>& tvecsRight,
                                      cv::Mat& leftToRightRotationMatrix,
                                      cv::Mat& leftToRightTranslationVector
                                     )
{
  double learningRate = 1;

  do
  {

    double cost = std::numeric_limits<double>::min() + 1;
    double previousCost = std::numeric_limits<double>::min();
    unsigned int numberOfIterations = 0;

    do
    {

      cost = InternalRenderingMonoIntrinsicCameraCalibration(modelFileName,
                                                             textureFileName,
                                                             leftImages,
                                                             rvecsLeft,
                                                             tvecsLeft,
                                                             learningRate,
                                                             intrinsicLeft,
                                                             distortionLeft
                                                             );

      std::cout << "RenderingMonoCameraCalibration:Cost=" << cost << ", Left Intrinsic="
                << intrinsicLeft.at<double>(0, 0) << " "
                << intrinsicLeft.at<double>(1, 1) << " "
                << intrinsicLeft.at<double>(0, 2) << " "
                << intrinsicLeft.at<double>(1, 2) << " "
                << distortionLeft.at<double>(0, 0) << " "
                << distortionLeft.at<double>(0, 1) << " "
                << distortionLeft.at<double>(0, 2) << " "
                << distortionLeft.at<double>(0, 3)
                << std::endl;

      cost = InternalRenderingMonoIntrinsicCameraCalibration(modelFileName,
                                                             textureFileName,
                                                             rightImages,
                                                             rvecsRight,
                                                             tvecsRight,
                                                             learningRate,
                                                             intrinsicRight,
                                                             distortionRight
                                                             );

      std::cout << "RenderingMonoCameraCalibration:Cost=" << cost << ", Right Intrinsic="
                << intrinsicRight.at<double>(0, 0) << " "
                << intrinsicRight.at<double>(1, 1) << " "
                << intrinsicRight.at<double>(0, 2) << " "
                << intrinsicRight.at<double>(1, 2) << " "
                << distortionRight.at<double>(0, 0) << " "
                << distortionRight.at<double>(0, 1) << " "
                << distortionRight.at<double>(0, 2) << " "
                << distortionRight.at<double>(0, 3)
                << std::endl;


      cost = InternalRenderingStereoCameraCalibration(modelFileName,
                                                      textureFileName,
                                                      leftImages,
                                                      rightImages,
                                                      intrinsicLeft,
                                                      distortionLeft,
                                                      intrinsicRight,
                                                      distortionRight,
                                                      learningRate,
                                                      rvecsLeft,
                                                      tvecsLeft,
                                                      rvecsRight,
                                                      tvecsRight,
                                                      leftToRightRotationMatrix,
                                                      leftToRightTranslationVector
                                                     );

      std::cout << "RenderingMonoCameraCalibration:Cost=" << cost << std::endl;

      numberOfIterations++;

    } while (cost - previousCost > 0.0001 && numberOfIterations < 100);

    learningRate /= 2.0;

  } while (learningRate > 0.1);
}

} // end namespace
