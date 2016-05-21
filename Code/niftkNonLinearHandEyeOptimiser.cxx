/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeOptimiser::NonLinearHandEyeOptimiser()
{
  m_CostFunction = niftk::NonLinearHandEyeCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearHandEyeOptimiser::~NonLinearHandEyeOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetModel(Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetHandMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearHandEyeOptimiser::Optimise(cv::Matx44d& modelToWorld,
                                           cv::Matx44d& handEye,
                                           cv::Mat& intrinsic,
                                           cv::Mat& distortion
                                          )
{

  if (intrinsic.rows != 3 || intrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic.cols << ", " << intrinsic.rows << ")";
  }
  if (distortion.rows != 1)
  {
    niftkNiftyCalThrow() << "Distortion vector should be a row vector.";
  }

  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(modelToWorld, modelToWorldRotationVector, modelToWorldTranslationVector);

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(handEye, handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearHandEyeCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  6               // model to world
                            + 6               // hand eye
                            + intrinsic.cols  // normally 4
                            + distortion.cols // could be 4..8
                           );

  // Set initial parameters.
  initialParameters[0] = handEyeRotationVector.at<double>(0, 0);
  initialParameters[1] = handEyeRotationVector.at<double>(0, 1);
  initialParameters[2] = handEyeRotationVector.at<double>(0, 2);
  initialParameters[3] = handEyeTranslationVector.at<double>(0, 0);
  initialParameters[4] = handEyeTranslationVector.at<double>(0, 1);
  initialParameters[5] = handEyeTranslationVector.at<double>(0, 2);
  initialParameters[6] = modelToWorldRotationVector.at<double>(0, 0);
  initialParameters[7] = modelToWorldRotationVector.at<double>(0, 1);
  initialParameters[8] = modelToWorldRotationVector.at<double>(0, 2);
  initialParameters[9] = modelToWorldTranslationVector.at<double>(0, 0);
  initialParameters[10] = modelToWorldTranslationVector.at<double>(0, 1);
  initialParameters[11] = modelToWorldTranslationVector.at<double>(0, 2);
  initialParameters[12] = intrinsic.at<double>(0, 0);
  initialParameters[13] = intrinsic.at<double>(1, 1);
  initialParameters[14] = intrinsic.at<double>(0, 2);
  initialParameters[15] = intrinsic.at<double>(1, 2);
  for (int c = 0; c < distortion.cols; c++)
  {
    initialParameters[16+c] = distortion.at<double>(0, c);
  }
  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(20000000);
  optimiser->SetGradientTolerance(0.000000005);
  optimiser->SetEpsilonFunction(0.000000005);
  optimiser->SetValueTolerance(0.000000005);

  niftk::NonLinearHandEyeCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);
  std::cout << "NonLinearHandEyeOptimiser: initial=" << initialParameters << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearHandEyeCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  handEyeRotationVector.at<double>(0, 0) = finalParameters[0];
  handEyeRotationVector.at<double>(0, 1) = finalParameters[1];
  handEyeRotationVector.at<double>(0, 2) = finalParameters[2];
  handEyeTranslationVector.at<double>(0, 0) = finalParameters[3];
  handEyeTranslationVector.at<double>(0, 1) = finalParameters[4];
  handEyeTranslationVector.at<double>(0, 2) = finalParameters[5];
  modelToWorldRotationVector.at<double>(0, 0) = finalParameters[6];
  modelToWorldRotationVector.at<double>(0, 1) = finalParameters[7];
  modelToWorldRotationVector.at<double>(0, 2) = finalParameters[8];
  modelToWorldTranslationVector.at<double>(0, 0) = finalParameters[9];
  modelToWorldTranslationVector.at<double>(0, 1) = finalParameters[10];
  modelToWorldTranslationVector.at<double>(0, 2) = finalParameters[11];
  intrinsic.at<double>(0, 0) = finalParameters[12];
  intrinsic.at<double>(1, 1) = finalParameters[13];
  intrinsic.at<double>(0, 2) = finalParameters[14];
  intrinsic.at<double>(1, 2) = finalParameters[15];
  for (int c = 0; c < distortion.cols; c++)
  {
    distortion.at<double>(0, c) = finalParameters[16+c];
  }
  modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearHandEyeCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);
  std::cout << "NonLinearHandEyeOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
