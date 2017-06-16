/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingCameraCalibration.h"
#include <niftkMatrixUtilities.h>
#include <Internal/niftkRenderingBasedMonoIntrinsicCostFunction.h>
#include <Internal/niftkRenderingBasedMonoExtrinsicCostFunction.h>
#include <Internal/niftkRenderingBasedStereoExtrinsicCostFunction.h>
#include <Internal/niftkCalibrationUtilities_p.h>
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
  itk::GradientDescentOptimizer::MeasureType currentValue = cost->GetValue(currentParams);

  unsigned int loopCounter = 0;

  while (currentValue > previousValue && fabs(currentValue - previousValue) > 0.001)
  {
    previousValue = currentValue;

    itk::GradientDescentOptimizer::Pointer opt = itk::GradientDescentOptimizer::New();
    opt->SetCostFunction(cost);
    opt->SetInitialPosition(currentParams);
    opt->SetLearningRate(learningRate);
    opt->SetMaximize(true);        // Because cost function is currently NMI.
    opt->SetNumberOfIterations(1); // ITK doesn't guarantee each step is an improvement.
    opt->StartOptimization();

    loopCounter++;
    currentValue = cost->GetValue(opt->GetCurrentPosition());

    if (currentValue > previousValue)
    {
      currentParams = opt->GetCurrentPosition();

      std::cerr << loopCounter << ": p=" << currentParams << ":" << previousValue << "," << currentValue << std::endl;
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

  if (currentValue > previousValue)
  {
    return currentValue; // as loop exited due to tolerance
  }
  else
  {
    return previousValue; // as loop exited due to no improvement.
  }
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
  win->DoubleBufferOff();
  win->GetInteractor()->Disable();

  niftk::RenderingBasedMonoExtrinsicCostFunction::Pointer cost = niftk::RenderingBasedMonoExtrinsicCostFunction::New();
  cost->Initialise(win,
                   windowSize,
                   calibratedWindowSize,
                   modelFileName,
                   textureFileName,
                   images,
                   intrinsic,
                   distortion
                  );

  niftk::RenderingBasedMonoExtrinsicCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  for (int i = 0; i < images.size(); i++)
  {
    currentParams[i*6 + 0] = rvecs[i].at<double>(0, 0);
    currentParams[i*6 + 1] = rvecs[i].at<double>(0, 1);
    currentParams[i*6 + 2] = rvecs[i].at<double>(0, 2);
    currentParams[i*6 + 3] = tvecs[i].at<double>(0, 0);
    currentParams[i*6 + 4] = tvecs[i].at<double>(0, 1);
    currentParams[i*6 + 5] = tvecs[i].at<double>(0, 2);
  }

  itk::GradientDescentOptimizer::MeasureType previousValue = std::numeric_limits<double>::min();
  itk::GradientDescentOptimizer::MeasureType currentValue = cost->GetValue(currentParams);

  unsigned int loopCounter = 0;

  while (currentValue > previousValue && fabs(currentValue - previousValue) > 0.001)
  {
    previousValue = currentValue;

    itk::GradientDescentOptimizer::Pointer opt = itk::GradientDescentOptimizer::New();
    opt->SetCostFunction(cost);
    opt->SetInitialPosition(currentParams);
    opt->SetLearningRate(learningRate);
    opt->SetMaximize(true);        // Because cost function is currently NMI.
    opt->SetNumberOfIterations(1); // ITK doesn't guarantee each step is an improvement.
    opt->StartOptimization();

    loopCounter++;
    currentValue = cost->GetValue(opt->GetCurrentPosition());

    if (currentValue > previousValue)
    {
      currentParams = opt->GetCurrentPosition();

      std::cerr << loopCounter << ": p=" << currentParams << ":" << previousValue << "," << currentValue << std::endl;
    }
  }

  niftk::RenderingBasedMonoIntrinsicCostFunction::ParametersType final = currentParams;
  for (int i = 0; i < images.size(); i++)
  {
    rvecs[i].at<double>(0, 0) = final[i*6 + 0];
    rvecs[i].at<double>(0, 1) = final[i*6 + 1];
    rvecs[i].at<double>(0, 2) = final[i*6 + 2];
    tvecs[i].at<double>(0, 0) = final[i*6 + 3];
    tvecs[i].at<double>(0, 1) = final[i*6 + 4];
    tvecs[i].at<double>(0, 2) = final[i*6 + 5];
  }

  if (currentValue > previousValue)
  {
    return currentValue; // as loop exited due to tolerance
  }
  else
  {
    return previousValue; // as loop exited due to no improvement.
  }
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
  win->DoubleBufferOff();
  win->GetInteractor()->Disable();

  niftk::RenderingBasedStereoExtrinsicCostFunction::Pointer cost = niftk::RenderingBasedStereoExtrinsicCostFunction::New();
  cost->Initialise(win,
                   windowSize,
                   calibratedWindowSize,
                   modelFileName,
                   textureFileName,
                   leftImages,
                   rightImages,
                   intrinsicLeft,
                   distortionLeft,
                   intrinsicRight,
                   distortionRight
                  );

  niftk::RenderingBasedStereoExtrinsicCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, rvec);

  currentParams[0] = rvec.at<double>(0, 0);
  currentParams[1] = rvec.at<double>(0, 1);
  currentParams[2] = rvec.at<double>(0, 2);
  currentParams[3] = leftToRightTranslationVector.at<double>(0, 0);
  currentParams[4] = leftToRightTranslationVector.at<double>(0, 1);
  currentParams[5] = leftToRightTranslationVector.at<double>(0, 2);

  for (int i = 0; i < leftImages.size(); i++)
  {
    currentParams[(i+1)*6 + 0] = rvecsLeft[i].at<double>(0, 0);
    currentParams[(i+1)*6 + 1] = rvecsLeft[i].at<double>(0, 1);
    currentParams[(i+1)*6 + 2] = rvecsLeft[i].at<double>(0, 2);
    currentParams[(i+1)*6 + 3] = tvecsLeft[i].at<double>(0, 0);
    currentParams[(i+1)*6 + 4] = tvecsLeft[i].at<double>(0, 1);
    currentParams[(i+1)*6 + 5] = tvecsLeft[i].at<double>(0, 2);
  }

  itk::GradientDescentOptimizer::MeasureType previousValue = std::numeric_limits<double>::min();
  itk::GradientDescentOptimizer::MeasureType currentValue = cost->GetValue(currentParams);

  unsigned int loopCounter = 0;

  while (currentValue > previousValue && fabs(currentValue - previousValue) > 0.001)
  {
    previousValue = currentValue;

    itk::GradientDescentOptimizer::Pointer opt = itk::GradientDescentOptimizer::New();
    opt->SetCostFunction(cost);
    opt->SetInitialPosition(currentParams);
    opt->SetLearningRate(learningRate);
    opt->SetMaximize(true);        // Because cost function is currently NMI.
    opt->SetNumberOfIterations(1); // ITK doesn't guarantee each step is an improvement.
    opt->StartOptimization();

    loopCounter++;
    currentValue = cost->GetValue(opt->GetCurrentPosition());

    if (currentValue > previousValue)
    {
      currentParams = opt->GetCurrentPosition();

      std::cerr << loopCounter << ": p=" << currentParams << ":" << previousValue << "," << currentValue << std::endl;
    }
  }

  niftk::RenderingBasedStereoExtrinsicCostFunction::ParametersType final = currentParams;
  rvec.at<double>(0, 0) = final[0];
  rvec.at<double>(0, 1) = final[1];
  rvec.at<double>(0, 2) = final[2];
  cv::Rodrigues(rvec, leftToRightRotationMatrix);
  leftToRightTranslationVector.at<double>(0, 0) = final[3];
  leftToRightTranslationVector.at<double>(0, 1) = final[4];
  leftToRightTranslationVector.at<double>(0, 2) = final[5];

  for (int i = 0; i < leftImages.size(); i++)
  {
    rvecsLeft[i].at<double>(0, 0) = final[(i+1)*6 + 0];
    rvecsLeft[i].at<double>(0, 1) = final[(i+1)*6 + 1];
    rvecsLeft[i].at<double>(0, 2) = final[(i+1)*6 + 2];
    tvecsLeft[i].at<double>(0, 0) = final[(i+1)*6 + 3];
    tvecsLeft[i].at<double>(0, 1) = final[(i+1)*6 + 4];
    tvecsLeft[i].at<double>(0, 2) = final[(i+1)*6 + 5];
  }

  // Makes sure that right hand rvecs and tvecs are consistent.
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  if (currentValue > previousValue)
  {
    return currentValue; // as loop exited due to tolerance
  }
  else
  {
    return previousValue; // as loop exited due to no improvement.
  }
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
  double learningRate = 0.01;

  do
  {

    unsigned int loopCounter = 0;
    double previousValue = std::numeric_limits<double>::min();
    double currentValue = std::numeric_limits<double>::min() + 1;

    while (currentValue > previousValue && fabs(currentValue - previousValue) > 0.001)
    {

      previousValue = currentValue;

      currentValue = InternalRenderingMonoIntrinsicCameraCalibration(win,
                                                                     windowSize,
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

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, intrinsic done, c=" << currentValue
                << std::endl;

      currentValue = InternalRenderingMonoExtrinsicCameraCalibration(win,
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

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, extrinsic done, c=" << currentValue
                << std::endl;

      loopCounter++;

    }

    learningRate /= 2.0;

  } while (learningRate > 0.001);
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
  double learningRate = 0.01;

  do
  {

    unsigned int loopCounter = 0;
    double previousValue = std::numeric_limits<double>::min();
    double currentValue = std::numeric_limits<double>::min() + 1;

    while (currentValue > previousValue && fabs(currentValue - previousValue) > 0.001)
    {

      previousValue = currentValue;

      currentValue = InternalRenderingMonoIntrinsicCameraCalibration(win,
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

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, left intrinsic done, c=" << currentValue
                << std::endl;

      currentValue = InternalRenderingMonoIntrinsicCameraCalibration(win,
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
      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, right intrinsic done, c=" << currentValue
                << std::endl;

      currentValue = InternalRenderingStereoCameraCalibration(win,
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

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", stereo, extrinsic done, c=" << currentValue
                << std::endl;

      loopCounter++;
    }

    learningRate /= 2.0;

  } while (learningRate > 0.001);
}

} // end namespace
