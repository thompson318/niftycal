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
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double InternalRenderingMonoIntrinsicCameraCalibration(vtkRenderWindow* win,
                                                       const cv::Size2i& windowSize,
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

  win->DoubleBufferOff();
  win->GetInteractor()->Disable();

  niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer cost = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
  cost->Initialise(win,
                   windowSize,
                   calibratedWindowSize,
                   modelFileName,
                   textureFileName,
                   images,
                   rvecs,
                   tvecs
                  );

  niftk::RenderingBasedMonoIntrinsicCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  currentParams[0] = intrinsic.at<double>(0, 0);
  currentParams[1] = intrinsic.at<double>(1, 1);
  currentParams[2] = intrinsic.at<double>(0, 2);
  currentParams[3] = intrinsic.at<double>(1, 2);
  currentParams[4] = distortion.at<double>(0, 0);
  currentParams[5] = distortion.at<double>(0, 1);
  currentParams[6] = distortion.at<double>(0, 2);
  currentParams[7] = distortion.at<double>(0, 3);

  itk::GradientDescentOptimizer::MeasureType previousValue = std::numeric_limits<double>::min();
  itk::GradientDescentOptimizer::MeasureType currentValue = std::numeric_limits<double>::min() + 1;

  while (currentValue > previousValue)
  {
    previousValue = currentValue;

    itk::GradientDescentOptimizer::Pointer opt = itk::GradientDescentOptimizer::New();
    opt->SetCostFunction(cost);
    opt->SetInitialPosition(currentParams);
    opt->SetLearningRate(learningRate);
    opt->SetMaximize(true);
    opt->SetNumberOfIterations(1); // ITK doesn't guarantee each step is an improvement.
    opt->StartOptimization();

    currentValue = opt->GetValue();

    if (currentValue > previousValue)
    {
      currentParams = opt->GetCurrentPosition();

      std::cout << "p=" << currentParams << ":" << currentValue << std::endl;
    }
  }

  niftk::RenderingBasedMonoIntrinsicCostFunction::ParametersType final = currentParams;
  intrinsic.at<double>(0, 0) = final[0];
  intrinsic.at<double>(1, 1) = final[1];
  intrinsic.at<double>(0, 2) = final[2];
  intrinsic.at<double>(1, 2) = final[3];
  distortion.at<double>(0, 0) = final[4];
  distortion.at<double>(0, 1) = final[5];
  distortion.at<double>(0, 2) = final[6];
  distortion.at<double>(0, 3) = final[7];

  return previousValue;
}


//-----------------------------------------------------------------------------
double InternalRenderingMonoExtrinsicCameraCalibration(vtkRenderWindow* win,
                                                       const cv::Size2i& windowSize,
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
double InternalRenderingStereoCameraCalibration(vtkRenderWindow* win,
                                                const cv::Size2i& windowSize,
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
void RenderingMonoCameraCalibration(vtkRenderWindow* win,
                                    const cv::Size2i& windowSize,
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
  double learningRate = 0.005;

  do
  {

    double previousValue = std::numeric_limits<double>::min();
    double currentValue = std::numeric_limits<double>::min() + 1;
    unsigned int numberOfIterations = 0;

    while (currentValue > previousValue && numberOfIterations < 1)
    {

      previousValue = currentValue;

      cv::Mat tmpIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
      intrinsic.copyTo(tmpIntrinsic);

      cv::Mat tmpDistortion = cv::Mat::zeros(1, 5, CV_64FC1);
      distortion.copyTo(tmpDistortion);

      currentValue = InternalRenderingMonoIntrinsicCameraCalibration(win,
                                                                     windowSize,
                                                                     calibratedWindowSize,
                                                                     modelFileName,
                                                                     textureFileName,
                                                                     images,
                                                                     rvecs,
                                                                     tvecs,
                                                                     learningRate,
                                                                     tmpIntrinsic,
                                                                     tmpDistortion
                                                                     );


      std::cout << "RenderingMonoCameraCalibration:Cost=" << currentValue
                << ", Learning=" << learningRate
                << ", Intrinsic="
                << intrinsic.at<double>(0, 0) << " "
                << intrinsic.at<double>(1, 1) << " "
                << intrinsic.at<double>(0, 2) << " "
                << intrinsic.at<double>(1, 2) << " "
                << distortion.at<double>(0, 0) << " "
                << distortion.at<double>(0, 1) << " "
                << distortion.at<double>(0, 2) << " "
                << distortion.at<double>(0, 3)
                << std::endl;
/*
      cost = InternalRenderingMonoExtrinsicCameraCalibration(win,
                                                             windowSize,
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
*/
      numberOfIterations++;

    }

    learningRate /= 2.0;

  } while (learningRate > 0.0001);
}


//-----------------------------------------------------------------------------
void RenderingStereoCameraCalibration(vtkRenderWindow* win,
                                      const cv::Size2i& windowSize,
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
  double learningRate = 0.001;

  do
  {

    double cost = std::numeric_limits<double>::min() + 1;
    double previousCost = std::numeric_limits<double>::min();
    unsigned int numberOfIterations = 0;

    do
    {

      cost = InternalRenderingMonoIntrinsicCameraCalibration(win,
                                                             windowSize,
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

      cost = InternalRenderingMonoIntrinsicCameraCalibration(win,
                                                             windowSize,
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


      cost = InternalRenderingStereoCameraCalibration(win,
                                                      windowSize,
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

    } while (previousCost - cost > 0.0001 && numberOfIterations < 20);

    learningRate /= 2.0;

  } while (learningRate > 0.001);
}

} // end namespace
