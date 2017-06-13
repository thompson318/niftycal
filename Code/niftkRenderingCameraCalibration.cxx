/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingCameraCalibration.h"
#include <Internal/niftkRenderingBasedMonoIntrinsicCostFunction.h>
#include <itkGradientDescentOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double InternalRenderingMonoIntrinsicCameraCalibration(const cv::Size2i& windowSize,
                                                       const cv::Size2i& calibratedWindowSize,
                                                       const std::string& modelFileName,
                                                       const std::string& textureFileName,
                                                       const std::vector<cv::Mat>& images,
                                                       const std::vector<cv::Mat>& rvecs,
                                                       const std::vector<cv::Mat>& tvecs,
                                                       const double& learningRate,
                                                       cv::Mat& intrinsic,
                                                       cv::Mat& distortion
                                                      )
{
  niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer cost = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
  cost->Initialise(windowSize,
                   calibratedWindowSize,
                   modelFileName,
                   textureFileName,
                   images,
                   rvecs,
                   tvecs,
                   intrinsic,
                   distortion
                   );

  niftk::RenderingBasedMonoIntrinsicCostFunction::ParametersType initial;
  initial.SetSize(cost->GetNumberOfParameters());

  initial[0] = intrinsic.at<double>(0, 0);
  initial[1] = intrinsic.at<double>(1, 1);
  initial[2] = intrinsic.at<double>(0, 2);
  initial[3] = intrinsic.at<double>(1, 2);
  initial[4] = distortion.at<double>(0, 0);
  initial[5] = distortion.at<double>(0, 1);
  initial[6] = distortion.at<double>(0, 2);
  initial[7] = distortion.at<double>(0, 3);

  itk::GradientDescentOptimizer::Pointer opt = itk::GradientDescentOptimizer::New();
  opt->SetCostFunction(cost);
  opt->SetInitialPosition(initial);
  opt->SetLearningRate(learningRate);
  opt->SetMaximize(true);
  opt->SetNumberOfIterations(20);
  opt->StartOptimization();

  niftk::RenderingBasedMonoIntrinsicCostFunction::ParametersType final = opt->GetCurrentPosition();
  intrinsic.at<double>(0, 0) = final[0];
  intrinsic.at<double>(1, 1) = final[1];
  intrinsic.at<double>(0, 2) = final[2];
  intrinsic.at<double>(1, 2) = final[3];
  distortion.at<double>(0, 0) = final[4];
  distortion.at<double>(0, 1) = final[5];
  distortion.at<double>(0, 2) = final[6];
  distortion.at<double>(0, 3) = final[7];

  return opt->GetValue();
}


//-----------------------------------------------------------------------------
double InternalRenderingMonoExtrinsicCameraCalibration(const cv::Size2i& windowSize,
                                                       const cv::Size2i& calibratedWindowSize,
                                                       const std::string& modelFileName,
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
double InternalRenderingStereoCameraCalibration(const cv::Size2i& windowSize,
                                                const cv::Size2i& calibratedWindowSize,
                                                const std::string& modelFileName,
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
void RenderingMonoCameraCalibration(const cv::Size2i& windowSize,
                                    const cv::Size2i& calibratedWindowSize,
                                    const std::string& modelFileName,
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

      cost = InternalRenderingMonoIntrinsicCameraCalibration(windowSize,
                                                             calibratedWindowSize,
                                                             modelFileName,
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

      cost = InternalRenderingMonoExtrinsicCameraCalibration(windowSize,
                                                             calibratedWindowSize,
                                                             modelFileName,
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

    } while (cost - previousCost > 0.0001 && numberOfIterations < 20);

    learningRate /= 2.0;

  } while (learningRate > 0.1);
}


//-----------------------------------------------------------------------------
void RenderingStereoCameraCalibration(const cv::Size2i& windowSize,
                                      const cv::Size2i& calibratedWindowSize,
                                      const std::string& modelFileName,
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

      cost = InternalRenderingMonoIntrinsicCameraCalibration(windowSize,
                                                             calibratedWindowSize,
                                                             modelFileName,
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

      cost = InternalRenderingMonoIntrinsicCameraCalibration(windowSize,
                                                             calibratedWindowSize,
                                                             modelFileName,
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


      cost = InternalRenderingStereoCameraCalibration(windowSize,
                                                      calibratedWindowSize,
                                                      modelFileName,
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

    } while (cost - previousCost > 0.0001 && numberOfIterations < 20);

    learningRate /= 2.0;

  } while (learningRate > 0.1);
}

} // end namespace
