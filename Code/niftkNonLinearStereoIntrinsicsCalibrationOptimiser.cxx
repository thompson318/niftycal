/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoIntrinsicsCalibrationOptimiser.h"
#include "niftkNonLinearStereoIntrinsicsCalibrationCostFunction.h"
#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationOptimiser::NonLinearStereoIntrinsicsCalibrationOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationOptimiser::~NonLinearStereoIntrinsicsCalibrationOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationOptimiser::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                                  std::vector<cv::Mat>* const tvecsLeft,
                                                                  cv::Mat* const leftToRightRotationMatrix,
                                                                  cv::Mat* const leftToRightTranslationVector
                                                                 )
{
  m_CostFunction->SetExtrinsics(rvecsLeft,
                                tvecsLeft,
                                leftToRightRotationMatrix,
                                leftToRightTranslationVector
                               );
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationOptimiser::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                            cv::Mat* const rightDistortion
                                                                           )
{
  m_CostFunction->SetDistortionParameters(leftDistortion,
                                          rightDistortion
                                         );
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoIntrinsicsCalibrationOptimiser::Optimise(cv::Mat& leftIntrinsic,
                                                               cv::Mat& rightIntrinsic
                                                              )
{
  if (leftIntrinsic.rows != 3 || leftIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic.cols << ", " << leftIntrinsic.rows << ")";
  }

  if (rightIntrinsic.rows != 3 || rightIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic.cols << ", " << rightIntrinsic.rows << ")";
  }

  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  4  // left intrinsic
                            + 4  // right intrinsic
                           );

  // Set initial parameters.
  int counter = 0;
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 2);

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(1000);
  optimiser->SetGradientTolerance(0.0001);
  optimiser->SetEpsilonFunction(0.0001);
  optimiser->SetValueTolerance(0.0001);

  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::MeasureType initialValues =
    m_CostFunction->GetValue(initialParameters);

  double initialRMS = m_CostFunction->GetRMS(initialValues);

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (int i = 0; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoIntrinsicsCalibrationOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }

  counter = 0;
  leftIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  leftIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 2) = finalParameters[counter++];

  std::cout << "NonLinearStereoIntrinsicsCalibrationOptimiser: initial rms=" << initialRMS
            << ", final rms=" << finalRMS
            << ", diff=" << finalRMS - initialRMS
            << std::endl;

  return finalRMS;
}

} // end namespace
