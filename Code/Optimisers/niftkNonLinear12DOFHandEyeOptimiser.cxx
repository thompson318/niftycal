/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinear12DOFHandEyeOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinear12DOFHandEyeOptimiser::NonLinear12DOFHandEyeOptimiser()
{
  m_CostFunction = niftk::NonLinear12DOFHandEyeCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinear12DOFHandEyeOptimiser::~NonLinear12DOFHandEyeOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinear12DOFHandEyeOptimiser::SetModel(Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinear12DOFHandEyeOptimiser::SetPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinear12DOFHandEyeOptimiser::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetHandMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinear12DOFHandEyeOptimiser::SetIntrinsic(cv::Mat* const intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinear12DOFHandEyeOptimiser::SetDistortion(cv::Mat* const distortion)
{
  m_CostFunction->SetDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinear12DOFHandEyeOptimiser::Optimise(cv::Matx44d& modelToWorld, cv::Matx44d& handEye)
{
  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(modelToWorld, modelToWorldRotationVector, modelToWorldTranslationVector);

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(handEye, handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinear12DOFHandEyeCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  6 // model to world
                            + 6 // hand eye
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

  niftk::NonLinear12DOFHandEyeCostFunction::MeasureType initialValues =
      m_CostFunction->GetValue(initialParameters);

  double initialRMS = m_CostFunction->GetRMS(initialValues);
  std::cout << "NonLinear12DOFHandEyeOptimiser: initial=" << initialParameters
            << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinear12DOFHandEyeCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
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

  modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinear12DOFHandEyeCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);
  std::cout << "NonLinear12DOFHandEyeOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  std::cout << "NonLinear12DOFHandEyeOptimiser: hand-eye="
            << handEyeRotationVector.at<double>(0, 0) << ", "
            << handEyeRotationVector.at<double>(0, 1) << ", "
            << handEyeRotationVector.at<double>(0, 2) << ", "
            << handEyeTranslationVector.at<double>(0, 0) << ", "
            << handEyeTranslationVector.at<double>(0, 1) << ", "
            << handEyeTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinear12DOFHandEyeOptimiser: model-to-world="
            << modelToWorldRotationVector.at<double>(0, 0) << ", "
            << modelToWorldRotationVector.at<double>(0, 1) << ", "
            << modelToWorldRotationVector.at<double>(0, 2) << ", "
            << modelToWorldTranslationVector.at<double>(0, 0) << ", "
            << modelToWorldTranslationVector.at<double>(0, 1) << ", "
            << modelToWorldTranslationVector.at<double>(0, 2) << std::endl;

  std::cout << "NonLinear12DOFHandEyeOptimiser: rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
