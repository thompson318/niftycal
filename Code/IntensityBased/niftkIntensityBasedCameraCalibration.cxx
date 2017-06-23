/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIntensityBasedCameraCalibration.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <Internal/niftkCalibrationUtilities_p.h>
#include <itkGradientDescentOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double InternalGradientDescentOptimisation(niftk::IntensityBasedCostFunction::ParametersType& currentParams,
                                           niftk::IntensityBasedCostFunction::Pointer& cost,
                                           const double& learningRate
                                          )
{
  itk::GradientDescentOptimizer::MeasureType previousValue = std::numeric_limits<double>::min();
  itk::GradientDescentOptimizer::MeasureType currentValue = cost->GetValue(currentParams);

  unsigned int loopCounter = 0;

  while (currentValue > previousValue
         && (fabs(currentValue - previousValue) > 0.0001)
        )
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
double InternalIntensityBasedMonoIntrinsicCameraCalibration(niftk::IntensityBasedCostFunction::Pointer& cost,
                                                            const double& learningRate,
                                                            cv::Mat& intrinsic,
                                                            cv::Mat& distortion
                                                           )
{

  if (cost->GetNumberOfParameters() != 9)
  {
    niftkNiftyCalThrow() << "Expected 9 parameters, but cost function wants " << cost->GetNumberOfParameters();
  }

  niftk::IntensityBasedCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  currentParams[0] = intrinsic.at<double>(0, 0);
  currentParams[1] = intrinsic.at<double>(1, 1);
  currentParams[2] = intrinsic.at<double>(0, 2);
  currentParams[3] = intrinsic.at<double>(1, 2);
  currentParams[4] = distortion.at<double>(0, 0);
  currentParams[5] = distortion.at<double>(0, 1);
  currentParams[6] = distortion.at<double>(0, 2);
  currentParams[7] = distortion.at<double>(0, 3);
  currentParams[8] = distortion.at<double>(0, 4);

  double finalCost = InternalGradientDescentOptimisation(currentParams, cost, learningRate);

  intrinsic.at<double>(0, 0) = currentParams[0];
  intrinsic.at<double>(1, 1) = currentParams[1];
  intrinsic.at<double>(0, 2) = currentParams[2];
  intrinsic.at<double>(1, 2) = currentParams[3];
  distortion.at<double>(0, 0) = currentParams[4];
  distortion.at<double>(0, 1) = currentParams[5];
  distortion.at<double>(0, 2) = currentParams[6];
  distortion.at<double>(0, 3) = currentParams[7];
  distortion.at<double>(0, 4) = currentParams[8];

  return finalCost;
}


//-----------------------------------------------------------------------------
double InternalIntensityBasedMonoExtrinsicCameraCalibration(niftk::IntensityBasedCostFunction::Pointer cost,
                                                            const double& learningRate,
                                                            std::vector<cv::Mat>& rvecs,
                                                            std::vector<cv::Mat>& tvecs
                                                           )
{
  int expectedNumberOfParameters = rvecs.size() * 6;

  if (cost->GetNumberOfParameters() != expectedNumberOfParameters)
  {
    niftkNiftyCalThrow() << "Expected " << expectedNumberOfParameters
                         << " parameters, but cost function wants " << cost->GetNumberOfParameters();
  }

  if (rvecs.size() != tvecs.size())
  {
    niftkNiftyCalThrow() << "Unequal number of rotation vectors (" << rvecs.size()
                         << ") and translation vectors (" << tvecs.size() << ")";
  }

  niftk::IntensityBasedCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  for (int i = 0; i < rvecs.size(); i++)
  {
    currentParams[i*6 + 0] = rvecs[i].at<double>(0, 0);
    currentParams[i*6 + 1] = rvecs[i].at<double>(0, 1);
    currentParams[i*6 + 2] = rvecs[i].at<double>(0, 2);
    currentParams[i*6 + 3] = tvecs[i].at<double>(0, 0);
    currentParams[i*6 + 4] = tvecs[i].at<double>(0, 1);
    currentParams[i*6 + 5] = tvecs[i].at<double>(0, 2);
  }

  double finalCost = InternalGradientDescentOptimisation(currentParams, cost, learningRate);

  for (int i = 0; i < rvecs.size(); i++)
  {
    rvecs[i].at<double>(0, 0) = currentParams[i*6 + 0];
    rvecs[i].at<double>(0, 1) = currentParams[i*6 + 1];
    rvecs[i].at<double>(0, 2) = currentParams[i*6 + 2];
    tvecs[i].at<double>(0, 0) = currentParams[i*6 + 3];
    tvecs[i].at<double>(0, 1) = currentParams[i*6 + 4];
    tvecs[i].at<double>(0, 2) = currentParams[i*6 + 5];
  }

  return finalCost;
}


//-----------------------------------------------------------------------------
double InternalRenderingStereoCameraCalibration(niftk::IntensityBasedCostFunction::Pointer cost,
                                                const double& learningRate,
                                                std::vector<cv::Mat>& rvecsLeft,
                                                std::vector<cv::Mat>& tvecsLeft,
                                                std::vector<cv::Mat>& rvecsRight,
                                                std::vector<cv::Mat>& tvecsRight,
                                                cv::Mat& leftToRightRotationMatrix,
                                                cv::Mat& leftToRightTranslationVector
                                               )
{

  if (rvecsLeft.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left rotation vectors (" << rvecsLeft.size()
                         << ") and left translation vectors (" << tvecsLeft.size() << ")";
  }

  if (rvecsLeft.size() != rvecsRight.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left rotation vectors (" << rvecsLeft.size()
                         << ") and right rotation vectors (" << rvecsRight.size() << ")";
  }

  if (rvecsLeft.size() != tvecsRight.size())
  {
    niftkNiftyCalThrow() << "Unequal number of left rotation vectors (" << rvecsLeft.size()
                         << ") and right rotation vectors (" << tvecsRight.size() << ")";
  }

  niftk::IntensityBasedCostFunction::ParametersType currentParams;
  currentParams.SetSize(cost->GetNumberOfParameters());

  cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, rvec);

  currentParams[0] = rvec.at<double>(0, 0);
  currentParams[1] = rvec.at<double>(0, 1);
  currentParams[2] = rvec.at<double>(0, 2);
  currentParams[3] = leftToRightTranslationVector.at<double>(0, 0);
  currentParams[4] = leftToRightTranslationVector.at<double>(0, 1);
  currentParams[5] = leftToRightTranslationVector.at<double>(0, 2);

  double finalCost = InternalGradientDescentOptimisation(currentParams, cost, learningRate);

  rvec.at<double>(0, 0) = currentParams[0];
  rvec.at<double>(0, 1) = currentParams[1];
  rvec.at<double>(0, 2) = currentParams[2];
  cv::Rodrigues(rvec, leftToRightRotationMatrix);

  leftToRightTranslationVector.at<double>(0, 0) = currentParams[3];
  leftToRightTranslationVector.at<double>(0, 1) = currentParams[4];
  leftToRightTranslationVector.at<double>(0, 2) = currentParams[5];

  // Makes sure that right hand rvecs and tvecs are consistent.
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  return finalCost;
}


//-----------------------------------------------------------------------------
void IntensityBasedMonoCameraCalibration(niftk::IntensityBasedCostFunction::Pointer intrinsicCostFunction,
                                         niftk::IntensityBasedCostFunction::Pointer extrinsicCostFunction,
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

    while (currentValue > previousValue
           && (fabs(currentValue - previousValue) > 0.0001)
          )
    {

      previousValue = currentValue;

      intrinsicCostFunction->SetActivated(true);
      extrinsicCostFunction->SetActivated(false);
      currentValue = InternalIntensityBasedMonoIntrinsicCameraCalibration(intrinsicCostFunction,
                                                                          learningRate,
                                                                          intrinsic,
                                                                          distortion
                                                                         );

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, intrinsic done, c=" << currentValue
                << std::endl;

      intrinsicCostFunction->SetActivated(false);
      extrinsicCostFunction->SetActivated(true);
      currentValue = InternalIntensityBasedMonoExtrinsicCameraCalibration(extrinsicCostFunction,
                                                                          learningRate,
                                                                          rvecs,
                                                                          tvecs
                                                                          );
      extrinsicCostFunction->SetActivated(false);

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
void IntensityBasedStereoCameraCalibration(niftk::IntensityBasedCostFunction::Pointer intrinsicLeftCostFunction,
                                           niftk::IntensityBasedCostFunction::Pointer intrinsicRightCostFunction,
                                           niftk::IntensityBasedCostFunction::Pointer extrinsicLeftCostFunction,
                                           niftk::IntensityBasedCostFunction::Pointer stereoExtrinsicCostFunction,
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

    while (currentValue > previousValue
           && (fabs(currentValue - previousValue) > 0.0001)
          )
    {

      previousValue = currentValue;

      intrinsicLeftCostFunction->SetActivated(true);
      intrinsicRightCostFunction->SetActivated(false);
      extrinsicLeftCostFunction->SetActivated(false);
      stereoExtrinsicCostFunction->SetActivated(false);
      currentValue = InternalIntensityBasedMonoIntrinsicCameraCalibration(intrinsicLeftCostFunction,
                                                                          learningRate,
                                                                          intrinsicLeft,
                                                                          distortionLeft
                                                                         );
      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, left intrinsic done, i=" << std::endl << intrinsicLeft << std::endl
                << "d=" << distortionLeft
                << ", c=" <<  currentValue
                << std::endl;

      intrinsicLeftCostFunction->SetActivated(false);
      intrinsicRightCostFunction->SetActivated(true);
      extrinsicLeftCostFunction->SetActivated(false);
      stereoExtrinsicCostFunction->SetActivated(false);
      currentValue = InternalIntensityBasedMonoIntrinsicCameraCalibration(intrinsicRightCostFunction,
                                                                          learningRate,
                                                                          intrinsicRight,
                                                                          distortionRight
                                                                         );


      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, right intrinsic done, i=" << std::endl << intrinsicRight << std::endl
                << "d=" << distortionRight
                << ", c=" <<  currentValue
                << std::endl;

      intrinsicLeftCostFunction->SetActivated(false);
      intrinsicRightCostFunction->SetActivated(false);
      extrinsicLeftCostFunction->SetActivated(true);
      stereoExtrinsicCostFunction->SetActivated(false);
      currentValue = InternalIntensityBasedMonoExtrinsicCameraCalibration(extrinsicLeftCostFunction,
                                                                          learningRate,
                                                                          rvecsLeft,
                                                                          tvecsLeft
                                                                         );


      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", mono, left extrinsic done, c=" <<  currentValue
                << std::endl;

      intrinsicLeftCostFunction->SetActivated(false);
      intrinsicRightCostFunction->SetActivated(false);
      extrinsicLeftCostFunction->SetActivated(false);
      stereoExtrinsicCostFunction->SetActivated(true);
      currentValue = InternalRenderingStereoCameraCalibration(stereoExtrinsicCostFunction,
                                                              learningRate,
                                                              rvecsLeft,
                                                              tvecsLeft,
                                                              rvecsRight,
                                                              tvecsRight,
                                                              leftToRightRotationMatrix,
                                                              leftToRightTranslationVector
                                                             );

      stereoExtrinsicCostFunction->SetActivated(false);

      std::cerr << loopCounter
                << ", l=" << learningRate
                << ", stereo, extrinsic done, t=" << leftToRightTranslationVector
                << ", c=" << currentValue
                << std::endl;

      loopCounter++;
    }

    learningRate /= 2.0;

  } while (learningRate > 0.001);
}

} // end namespace
