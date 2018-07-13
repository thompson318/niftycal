/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMaltiHandEyeOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMaltiHandEyeOptimiser::NonLinearMaltiHandEyeOptimiser()
{
  m_CostFunction = niftk::NonLinearMaltiHandEyeCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearMaltiHandEyeOptimiser::~NonLinearMaltiHandEyeOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearMaltiHandEyeOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiHandEyeOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiHandEyeOptimiser::SetHandMatrices(const std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetHandMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearMaltiHandEyeOptimiser::Optimise(cv::Matx44d& modelToWorld,
                                                cv::Matx44d& handEye,
                                                cv::Mat& intrinsic,
                                                cv::Mat& distortion
                                               )
{

  if (intrinsic.rows != 3 || intrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic.rows << ", " << intrinsic.cols << ")";
  }
  if (distortion.rows != 1 || distortion.cols != 5)
  {
    niftkNiftyCalThrow() << "Distortion vector should be 1x5, and its ("
                         << distortion.rows << ", " << distortion.cols << ")";
  }

  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(modelToWorld, modelToWorldRotationVector, modelToWorldTranslationVector);

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(handEye, handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearMaltiHandEyeCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  4 // intrinsic
                            + 5 // distortion
                            + 6 // hand eye
                            + 6 // model to world
                           );

  // Set initial parameters.
  initialParameters[0] = intrinsic.at<double>(0, 0);
  initialParameters[1] = intrinsic.at<double>(1, 1);
  initialParameters[2] = intrinsic.at<double>(0, 2);
  initialParameters[3] = intrinsic.at<double>(1, 2);
  initialParameters[4] = distortion.at<double>(0, 0);
  initialParameters[5] = distortion.at<double>(0, 1);
  initialParameters[6] = distortion.at<double>(0, 2);
  initialParameters[7] = distortion.at<double>(0, 3);
  initialParameters[8] = distortion.at<double>(0, 4);
  initialParameters[9] = handEyeRotationVector.at<double>(0, 0);
  initialParameters[10] = handEyeRotationVector.at<double>(0, 1);
  initialParameters[11] = handEyeRotationVector.at<double>(0, 2);
  initialParameters[12] = handEyeTranslationVector.at<double>(0, 0);
  initialParameters[13] = handEyeTranslationVector.at<double>(0, 1);
  initialParameters[14] = handEyeTranslationVector.at<double>(0, 2);
  initialParameters[15] = modelToWorldRotationVector.at<double>(0, 0);
  initialParameters[16] = modelToWorldRotationVector.at<double>(0, 1);
  initialParameters[17] = modelToWorldRotationVector.at<double>(0, 2);
  initialParameters[18] = modelToWorldTranslationVector.at<double>(0, 0);
  initialParameters[19] = modelToWorldTranslationVector.at<double>(0, 1);
  initialParameters[20] = modelToWorldTranslationVector.at<double>(0, 2);

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(100);
  optimiser->SetGradientTolerance(0.0000001);
  optimiser->SetEpsilonFunction(0.0000001);
  optimiser->SetValueTolerance(0.0000001);

  niftk::NonLinearMaltiHandEyeCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);

  std::cout << "NonLinearMaltiHandEyeOptimiser: initial=" << initialParameters << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearMaltiHandEyeCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  intrinsic.at<double>(0, 0) = finalParameters[0];
  intrinsic.at<double>(1, 1) = finalParameters[1];
  intrinsic.at<double>(0, 2) = finalParameters[2];
  intrinsic.at<double>(1, 2) = finalParameters[3];
  distortion.at<double>(0, 0) = finalParameters[4];
  distortion.at<double>(0, 1) = finalParameters[5];
  distortion.at<double>(0, 2) = finalParameters[6];
  distortion.at<double>(0, 3) = finalParameters[7];
  distortion.at<double>(0, 4) = finalParameters[8];
  handEyeRotationVector.at<double>(0, 0) = finalParameters[9];
  handEyeRotationVector.at<double>(0, 1) = finalParameters[10];
  handEyeRotationVector.at<double>(0, 2) = finalParameters[11];
  handEyeTranslationVector.at<double>(0, 0) = finalParameters[12];
  handEyeTranslationVector.at<double>(0, 1) = finalParameters[13];
  handEyeTranslationVector.at<double>(0, 2) = finalParameters[14];
  modelToWorldRotationVector.at<double>(0, 0) = finalParameters[15];
  modelToWorldRotationVector.at<double>(0, 1) = finalParameters[16];
  modelToWorldRotationVector.at<double>(0, 2) = finalParameters[17];
  modelToWorldTranslationVector.at<double>(0, 0) = finalParameters[18];
  modelToWorldTranslationVector.at<double>(0, 1) = finalParameters[19];
  modelToWorldTranslationVector.at<double>(0, 2) = finalParameters[20];

  modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearMaltiHandEyeCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  std::cout << "NonLinearMaltiHandEyeOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  std::cout << "NonLinearMaltiHandEyeOptimiser: hand-eye="
            << handEyeRotationVector.at<double>(0, 0) << ", "
            << handEyeRotationVector.at<double>(0, 1) << ", "
            << handEyeRotationVector.at<double>(0, 2) << ", "
            << handEyeTranslationVector.at<double>(0, 0) << ", "
            << handEyeTranslationVector.at<double>(0, 1) << ", "
            << handEyeTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinearMaltiHandEyeOptimiser: model-to-world="
            << modelToWorldRotationVector.at<double>(0, 0) << ", "
            << modelToWorldRotationVector.at<double>(0, 1) << ", "
            << modelToWorldRotationVector.at<double>(0, 2) << ", "
            << modelToWorldTranslationVector.at<double>(0, 0) << ", "
            << modelToWorldTranslationVector.at<double>(0, 1) << ", "
            << modelToWorldTranslationVector.at<double>(0, 2) << std::endl;

  std::cout << "NonLinearMaltiHandEyeOptimiser: rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
